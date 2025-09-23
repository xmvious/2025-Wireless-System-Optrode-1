/* 
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample (Binary + Framing enabled)
 *  // 이 파일은 BLE NUS를 이용해 UART-같은 통신을 구현하는 peripheral 예제이며,
 *  // 바이너리 프레이밍(헤더+페이로드)로 패킷을 주고받도록 확장된 버전이다.
 */
#include <zephyr/sys/byteorder.h>  // 엔디안(endianness) 변환 유틸: sys_cpu_to_le16, sys_le16_to_cpu, sys_put_le16 등
#include <stdlib.h>                // 문자열을 숫자로 변환(atoi 등), 메모리 관련 함수들

#include <uart_async_adapter.h>    // UART 드라이버를 async 방식으로 래핑해주는 어댑터(필요 시 사용)

#include <zephyr/types.h>          // Zephyr 공용 타입 정의(u8_t 등)
#include <zephyr/kernel.h>         // 커널 API(k_thread, k_fifo, k_work, k_sem 등)
#include <zephyr/drivers/uart.h>   // UART 드라이버 인터페이스
#include <zephyr/usb/usb_device.h> // USB CDC ACM 등 USB 장치 초기화용(PC와 가상 COM 포트 연결)

#include <zephyr/device.h>         // device tree 기반 디바이스 핸들 가져오기(DEVICE_DT_GET 등)
#include <zephyr/devicetree.h>     // devicetree 매크로/헬퍼
#include <soc.h>                   // SoC(칩) 레벨 레지스터/정의

#include <zephyr/bluetooth/bluetooth.h> // BLE 스택 기본 제어(bt_enable, adv 등)
#include <zephyr/bluetooth/uuid.h>      // BLE UUID 관련 헬퍼
#include <zephyr/bluetooth/gatt.h>      // GATT 서버/클라이언트 API
#include <zephyr/bluetooth/hci.h>       // HCI 레벨 정의(코드에서 직접 쓰이는 경우는 드묾)

#include <bluetooth/services/nus.h> // Nordic UART Service(NUS) 서버/클라 API

#include <dk_buttons_and_leds.h>    // nRF DK 보드의 버튼/LED 헬퍼
#include <zephyr/settings/settings.h>// settings(플래시)에 파라미터 저장/로드

#include <stdio.h>                  // 표준 입출력(printf 등)
#include <string.h>                 // 문자열/메모리 함수(memcpy, memset 등)
#include <zephyr/logging/log.h>     // 로그 모듈(LOG_INF/WRN/ERR 등)

#include <zephyr/drivers/gpio.h>    // GPIO 제어(핀 방향, set/reset)
#include <hal/nrf_gpio.h>           // nRF 하드웨어 레벨 GPIO 접근(고성능/특수기능 시)

#include <zephyr/drivers/adc.h>     // ADC 드라이버 상위 API
#include <nrfx_saadc.h>             // Nordic SAADC(ADC) 하위 HAL(고성능/세부 제어)

/* ============================================================
 *              프로젝트/프레이밍 공통 상수
 * ============================================================*/
#define LOG_MODULE_NAME peripheral_uart      // 로그 모듈 이름
LOG_MODULE_REGISTER(LOG_MODULE_NAME);        // 로그 모듈 등록(필요시 로그 레벨 설정 가능)

/* Frame header (센트럴과 동일) */
// 센트럴/퍼리 양쪽이 같은 헤더 포맷을 사용해 패킷을 구분하고 길이/채널 정보를 전달
#define FRAME_MAGIC             0xA5C3       // 프레임 식별자(매직 값)로 패킷 동기화에 사용
typedef struct __packed {
    uint16_t magic;        /* 0xA5C3 */      // 매직 값(LE). 잘못 수신 시 프레이밍 재동기화 힌트
    uint16_t seq;          /* LE */          // 시퀀스 번호. 패킷 순서 확인/유실 감지 등에 활용
    uint16_t payload_len;  /* LE */          // 페이로드 길이(바이트). MTU 조각 전 전체 길이
    uint8_t  ch_mask;      /* ex: 4채널 -> 0b1111 */ // 활성 채널 비트마스크(ADC 등 멀티채널 표기)
    uint8_t  flags;        /* bit0: payload is ASCII command (optional) */ // 페이로드 속성 플래그
} frame_hdr_t;                                // __packed로 패딩 없이 정확한 바이트 레이아웃 보장

#define DEFAULT_CH_MASK         0x0F          // 기본 4채널(0~3) 활성
#define FLAG_ASCII_PAYLOAD      0x01          // 페이로드가 ASCII Command임을 표시할 때 사용(선택)

/* ATT MTU (연결 후 갱신), NUS 한 번에 보낼 수 있는 최대 청크 = MTU - 3 */
// ATT MTU는 연결 협상으로 커질 수 있음. NUS는 1회 Notify에서 (MTU-3)바이트까지 전송 가능
static uint16_t att_mtu = 23;                 // 기본 BLE MTU(23). 교환되면 더 커질 수 있음
static inline uint16_t nus_max_chunk(void) { return (att_mtu > 3) ? (att_mtu - 3) : 20; } // 전송 가능한 청크 계산

/* ============================================================
 *                       기존 상수/정의
 * ============================================================*/
#define STACKSIZE               CONFIG_BT_NUS_THREAD_STACK_SIZE // NUS 스레드 스택 크기(Kconfig)
#define PRIORITY                7                                // 이 스레드 우선순위(숫자 작을수록 높음)

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME            // 광고에 사용할 장치 이름
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)        // 널문자 제외 길이

#define RUN_STATUS_LED          DK_LED1                          // 동작 상태 LED(깜박임)
#define RUN_LED_BLINK_INTERVAL  1000                             // ms 단위 깜박 주기
#define CON_STATUS_LED          DK_LED2                          // 연결 상태 LED(연결 시 ON)

#define KEY_PASSKEY_ACCEPT      DK_BTN1_MSK                      // 페어링 패스키 수락 버튼
#define KEY_PASSKEY_REJECT      DK_BTN2_MSK                      // 페어링 패스키 거절 버튼

/* UART buffer는 바이너리 스트림 고려하여 충분히 크게 */
// 바이너리/프레이밍 시 burst 전송이 있어 버퍼 크게 잡음
#ifndef CONFIG_BT_NUS_UART_BUFFER_SIZE
#define CONFIG_BT_NUS_UART_BUFFER_SIZE 512                       // 기본 UART 버퍼 크기(없으면 512로)
#endif
#define UART_BUF_SIZE           CONFIG_BT_NUS_UART_BUFFER_SIZE   // 실제 UART 버퍼 크기
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)                       // 버퍼 부족 시 재시도 지연
/* us 단위: 타임아웃 발생 시 드라이버가 조각을 밀어줌(플랫폼마다 상이) */
#define UART_WAIT_FOR_RX        50000                            // RX 타임아웃(마이크로초). 조각 flush 유도
/* flush 임계값 */
#define UART_FLUSH_THRESHOLD    (UART_BUF_SIZE - 16)             // 버퍼가 이만큼 차면 강제 전송/처리

/* GPIO: LED */
#define GPIO_PIN                NRF_GPIO_PIN_MAP(0,3)            // Port0 Pin3 사용(보드 LED 제어 예시)

/* ADC */
#define ADC_RESOLUTION          12                               // ADC 해상도(bit). 12-bit → 0~4095
#define NUM_CHANNELS            4                                // 동시 샘플링할 채널 수(예: 4채널)

/* ============================================================
 *                       전역 상태
 * ============================================================*/
struct uart_data_t {
    void    *fifo_reserved;               // k_fifo가 내부적으로 사용하는 포인터(첫 멤버로 필요)
    uint8_t  data[UART_BUF_SIZE];         // 수신/송신 데이터 버퍼
    uint16_t len;                         // 현재 저장된 데이터 길이
};

static struct uart_data_t *pending_buf;    // UART 수신 중 임시로 보류된 버퍼 포인터
static K_SEM_DEFINE(ble_init_ok, 0, 1);    // BLE 초기화 완료 신호 세마포어(0으로 시작, 최대 1)
static struct bt_conn *current_conn;       // 현재 활성 BLE 연결 핸들
static struct bt_conn *auth_conn;          // 인증/페어링 진행 중 연결 핸들

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart)); // DT에서 선택된 UART 디바이스
static struct k_work_delayable uart_work;  // 지연 실행 워크(주기적 flush 등 스케줄링)
static struct k_work ble_send_work;        // 즉시 실행 워크(BLE 전송 트리거 등)

static K_FIFO_DEFINE(fifo_uart_tx_data);   // UART TX 데이터 큐(생산자-소비자 패턴)
static K_FIFO_DEFINE(fifo_uart_rx_data);   // UART RX 데이터 큐

// 광고(Advertising) 데이터: 일반 모드 + BR/EDR 비활성, 장치 이름 포함
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)), // 플래그: 일반발견모드/BR-EDR 미지원
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),         // 전체 장치 이름 광고
};

// 스캔 응답(Scan Response) 데이터: NUS 128-bit UUID 포함(스캐너가 본 장치의 서비스 추정 가능)
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),                  // NUS UUID 광고
};

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter); // async 어댑터 인스턴스 생성(드라이버가 sync일 때 래핑)
#else
#define async_adapter NULL                   // 어댑터 미사용 시 NULL로 처리
#endif

/* ATT MTU 조회용 (peripheral 쪽은 교환 콜백 없으니 gatt_get_mtu를 수신 시점 등에서 사용) */
// 연결 직후 또는 Notification 전 bt_gatt_get_mtu(conn)로 현재 MTU를 반영하여 청크 크기 최적화
static inline void update_mtu_from_conn(struct bt_conn *conn) {
    if (!conn) return;                      // 널 가드
    uint16_t m = bt_gatt_get_mtu(conn);     // 연결별 현재 GATT MTU 조회
    if (m >= 23) att_mtu = m;               // 최소 23. 더 크면 업데이트
}

/* ============================================================
 *                       ADC 설정
 * ============================================================*/
static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc)); // DT에서 SAADC 디바이스 가져오기
static int16_t sample_buf[NUM_CHANNELS];                                // 샘플 저장 버퍼(채널당 1개 샘플)

// Adafruit Feather nRF52840 보드 기준: A2/A4/A5/A6 → AIN2/AIN4/AIN5/AIN6(참고 보드 핀맵)
static const uint8_t channel_ids[NUM_CHANNELS] = {2, 4, 5, 6};          // SAADC 채널 ID(논리)
static const nrf_saadc_input_t inputs[NUM_CHANNELS] = {
    NRF_SAADC_INPUT_AIN2,                                               // 물리 입력 AIN2
    NRF_SAADC_INPUT_AIN4,                                               // 물리 입력 AIN4
    NRF_SAADC_INPUT_AIN5,                                               // 물리 입력 AIN5
    NRF_SAADC_INPUT_AIN6,                                               // 물리 입력 AIN6
};

static struct adc_channel_cfg adc_cfg; // 채널 설정 구조체(반복 사용으로 스택/코드 절감)
static struct adc_sequence sequence = {
    .buffer      = sample_buf,                // 변환 결과가 저장될 버퍼
    .buffer_size = sizeof(sample_buf),        // 버퍼 크기(바이트)
    .resolution  = ADC_RESOLUTION,            // 해상도(12-bit)
    // .channels는 매 호출 전에 동적으로 채움
};

// 4채널을 한 번에 읽어 sample_buf에 저장하는 헬퍼 함수
static int adc_read_4ch(void)
{
    int err;                                  // 에러 코드 임시 저장
    uint32_t channels_mask = 0;               // 활성화할 채널 비트마스크(ADC 드라이버용)

    for (int i = 0; i < NUM_CHANNELS; i++) {  // 채널 수만큼 반복 설정
        adc_cfg = (struct adc_channel_cfg){    // 채널 개별 설정(초기화 리스트)
            .gain             = ADC_GAIN_1_6,          // 입력 gain(1/6): 풀스케일 전압 확대
            .reference        = ADC_REF_VDD_1_4,       // Vref = VDD/4 (내부 기준)
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,  // 표준 샘플 유지 시간
            .channel_id       = channel_ids[i],        // 논리 채널 번호
#if defined(CONFIG_ADC_NRFX_SAADC)
            .input_positive   = inputs[i],             // 물리 입력 매핑(SAADC 전용 필드)
#endif
        };
        err = adc_channel_setup(adc_dev, &adc_cfg);    // 해당 채널 설정을 드라이버에 등록
        if (err) {
            LOG_ERR("ADC channel setup failed for ch%d (err %d)", i, err); // 설정 실패 로그
            return err;                           // 즉시 에러 리턴
        }
        channels_mask |= BIT(channel_ids[i]);     // 설정된 채널을 마스크에 추가
    }

    sequence.channels = channels_mask;            // 이번 변환에서 사용할 채널 집합 지정
    err = adc_read(adc_dev, &sequence);           // 블로킹 방식으로 변환 수행(모든 채널 측정)
    if (err) {
        LOG_ERR("ADC read error: %d", err);       // 변환 실패 시 로그
        return err;                                // 에러 리턴
    }
    return 0;                                     // 성공(결과는 sample_buf에 채널 순서대로 존재)
}

/* ============================================================ 
 *                      NUS 전송 유틸 (청크 분할)
 * ============================================================*/
static K_SEM_DEFINE(nus_write_sem, 0, 1);        // NUS 전송 완료 신호를 기다릴 세마포어 (초기값 0, 최대 1)

static void nus_sent_cb(struct bt_conn *conn)    // NUS 데이터 전송 완료 시 불리는 콜백
{
    ARG_UNUSED(conn);                            // conn 인자는 사용하지 않음 (경고 제거용)
    k_sem_give(&nus_write_sem);                  // 세마포어를 주어 대기 중인 전송 루프를 깨움
}

static int nus_send_chunked(const uint8_t *p, uint16_t len) // 길이가 긴 데이터 → MTU 크기 단위로 잘라 전송
{
    while (len) {                                // 남은 데이터가 있을 동안 반복
        uint16_t n = MIN(nus_max_chunk(), len);  // 이번에 보낼 길이 = (MTU-3) 또는 남은 길이
        int err = bt_nus_send(current_conn, p, n);// NUS로 n바이트 전송
        if (err) return err;                     // 에러 발생 시 바로 리턴
        int s = k_sem_take(&nus_write_sem, K_MSEC(150)); // 전송 완료 콜백 대기 (최대 150ms)
        if (s) LOG_WRN("NUS send timeout");      // 제한시간 내 완료 안 되면 경고 로그
        p   += n;                                // 보낸 위치만큼 포인터 이동
        len -= n;                                // 남은 길이 줄이기
    }
    return 0;                                    // 모든 조각 전송 성공
}

static int send_frame_over_nus(const uint8_t *payload, uint16_t payload_len,
                               uint8_t ch_mask, uint8_t flags) // 프레임 헤더 + payload → NUS 전송
{
    if (!current_conn) return -ENOTCONN;         // 연결 없으면 에러 리턴

    const uint16_t total = sizeof(frame_hdr_t) + payload_len; // 총 길이 = 헤더 + 페이로드
    uint8_t *frame = k_malloc(total);            // 동적 메모리로 프레임 버퍼 할당
    if (!frame) return -ENOMEM;                  // 메모리 부족 시 에러 리턴

    static uint16_t seq = 0;                     // 시퀀스 번호(전역 static, 자동 증가)
    frame_hdr_t hdr;                             // 프레임 헤더 준비
    hdr.magic       = sys_cpu_to_le16(FRAME_MAGIC);      // 매직 값(LE 변환)
    hdr.seq         = sys_cpu_to_le16(seq++);            // 순차 증가하는 시퀀스 번호
    hdr.payload_len = sys_cpu_to_le16(payload_len);      // 페이로드 길이(LE)
    hdr.ch_mask     = ch_mask;                           // 채널 마스크
    hdr.flags       = flags;                             // 옵션 플래그

    memcpy(frame, &hdr, sizeof(hdr));           // 버퍼에 헤더 복사
    memcpy(frame + sizeof(hdr), payload, payload_len); // 이어서 페이로드 붙임

    update_mtu_from_conn(current_conn);          // 현재 연결의 MTU 갱신
    int err = nus_send_chunked(frame, total);    // 전체 프레임을 조각내어 NUS 전송
    k_free(frame);                               // 동적 할당 해제
    return err;                                  // 전송 결과 리턴
}

/* ============================================================
 *                      ADC → BLE 전송 (프레임)
 * ============================================================*/
static int send_adc_as_frame(void)               // ADC 4채널 샘플을 프레임으로 포장해 전송
{
    /* payload: 4ch * int16 (LE) */
    uint8_t payload[NUM_CHANNELS * sizeof(int16_t)]; // 4채널 × 2바이트 = 8바이트 버퍼
    for (int i = 0; i < NUM_CHANNELS; i++) {
        sys_put_le16((uint16_t)sample_buf[i], &payload[i * 2]); // 각 샘플을 LE 형식으로 기록
    }
    return send_frame_over_nus(payload, sizeof(payload), DEFAULT_CH_MASK, 0x00); // 프레임 전송
}

/* ============================================================
 *                      주기적 ADC 워크/타이머
 * ============================================================*/
static void read_adc_and_print_all(struct k_work *work) // 워크큐에서 실행될 ADC 읽기 함수
{
    ARG_UNUSED(work);                         // work 인자는 사용 안 함
    if (adc_read_4ch() == 0) {                // 4채널 ADC 읽기 성공 시
        /* 기본: 바이너리 프레임 전송 */
        int e = send_adc_as_frame();          // 샘플을 프레임으로 BLE 전송
        if (e) LOG_WRN("send_adc_as_frame err=%d", e); // 실패 시 경고 로그

        /* (옵션) 개발 중 확인용 텍스트 출력 */
        /* printk("ADC: %d, %d, %d, %d\n", sample_buf[0], sample_buf[1], sample_buf[2], sample_buf[3]); */
    }
}
K_WORK_DEFINE(adc_work, read_adc_and_print_all); // 위 함수를 워크 객체로 정의

static void adc_timer_handler(struct k_timer *timer_id) // 주기 타이머 만료 시 실행되는 콜백
{
    k_work_submit(&adc_work);                 // 워크큐에 adc_work 제출(비동기 실행)
}
K_TIMER_DEFINE(adc_timer, adc_timer_handler, NULL); // 타이머 객체 정의 (주기마다 ADC 읽기)

/* ============================================================
 *                 BLE 수신 → 프레임 언프레임/하위호환
 * ============================================================*/
static void handle_ascii_command(const char *received_string) // 텍스트 명령 처리 (on/off, 펄스 설정)
{
    /* 기존 on/off, pulse,period,duty 파싱 로직 그대로 */
    nrf_gpio_cfg_output(GPIO_PIN);            // GPIO 핀을 출력으로 설정

    if (strchr(received_string, ',') != NULL) { // 쉼표가 있으면 pulse,period,duty 형식
        int pulse = 0, period = 0, duty = 0;  // 파라미터 변수
        char buf[128];                        // 로컬 복사용 버퍼
        size_t n = MIN(sizeof(buf)-1, strlen(received_string));
        memcpy(buf, received_string, n); buf[n] = '\0'; // 문자열 안전 복사

        char *token = strtok(buf, ",");       // 첫 토큰 = pulse
        if (token) { pulse = atoi(token); token = strtok(NULL, ","); }
        if (token) { period = atoi(token); token = strtok(NULL, ","); }
        if (token) { duty = atoi(token); }    // 세 번째 토큰 = duty

        const float unit = 0.3125f;           // 시간 단위 변환(타이머 tick 단위 ms)
        int on_ms  = (int)(duty * unit);      // LED ON 시간(ms)
        int off_ms = (int)((period - duty) * unit); // LED OFF 시간(ms)
        for (int i=0; i<pulse; i++) {         // 지정된 횟수만큼 반복
            nrf_gpio_pin_write(GPIO_PIN, 1);  // LED ON
            k_msleep(on_ms);                  // on_ms 동안 대기
            nrf_gpio_pin_write(GPIO_PIN, 0);  // LED OFF
            k_msleep(off_ms);                 // off_ms 동안 대기
        }
        return;                               // 끝
    }

    static bool adc_timer_started = false;    // ADC 주기 타이머 실행 상태 플래그
    if (strcmp(received_string, "on\n") == 0 || strcmp(received_string, "on") == 0) {
        if (!adc_timer_started) {             // 아직 시작 안 했으면
            k_timer_start(&adc_timer, K_MSEC(10), K_MSEC(50)); // 10ms 뒤 시작, 50ms 주기
            adc_timer_started = true;
            printk("[PERIPHERAL] ADC sampling started.\n");
        }
    } else if (strcmp(received_string, "off\n") == 0 || strcmp(received_string, "off") == 0) {
        k_timer_stop(&adc_timer);             // 타이머 정지
        adc_timer_started = false;
        printk("[PERIPHERAL] ADC sampling stopped.\n");
    }
}

static void handle_payload_to_uart(const uint8_t *data, uint16_t len) // 바이너리 payload → UART로 그대로 흘리기
{
    /* 바이너리 그대로 UART로 내보냄 (개행 X) */
    for (uint16_t pos = 0; pos < len; ) {     // 남은 길이만큼 반복
        struct uart_data_t *tx = k_malloc(sizeof(*tx)); // 전송 버퍼 동적 할당
        if (!tx) { LOG_WRN("UART tx alloc fail"); return; } // 메모리 부족 시 경고
        size_t n = MIN(sizeof(tx->data), (size_t)(len - pos)); // 이번에 보낼 조각 크기
        memcpy(tx->data, &data[pos], n);      // 데이터 복사
        tx->len = (uint16_t)n;                // 길이 기록
        pos += n;                             // 포인터 이동
        int err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS); // UART 전송 요청
        if (err) {
            k_fifo_put(&fifo_uart_tx_data, tx); // 에러 시 큐에 넣어 나중에 처리
        } else {
            /* free는 TX_DONE에서 수행 → 여기선 해제 안 함 */
        }
    }
}

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len) // BLE 수신 콜백
{
    update_mtu_from_conn(conn);              // 연결된 conn의 MTU 업데이트

    /* 1) 프레임인지 검사 */
    if (len >= sizeof(frame_hdr_t)) {        // 최소한 헤더 크기 이상인지 확인
        frame_hdr_t hdr;                     // 헤더 구조체 임시
        memcpy(&hdr, data, sizeof(hdr));     // 수신 데이터에서 헤더 복사
        uint16_t magic = sys_le16_to_cpu(hdr.magic); // 매직 값 복원
        uint16_t plen  = sys_le16_to_cpu(hdr.payload_len); // payload 길이
        if (magic == FRAME_MAGIC && (sizeof(hdr) + plen) <= len) { // 매직값/길이 검증
            const uint8_t *payload = data + sizeof(hdr); // payload 위치 계산
            /* flags bit0가 1이면 payload를 ASCII 명령으로 취급 */
            if (hdr.flags & FLAG_ASCII_PAYLOAD) {
                char tmp[256];
                size_t n = MIN(sizeof(tmp)-1, (size_t)plen); // 안전 복사
                memcpy(tmp, payload, n); tmp[n] = '\0';
                handle_ascii_command(tmp);   // 문자열 명령 처리
            } else {
                /* 일반 바이너리 payload → UART로 전달 */
                handle_payload_to_uart(payload, plen);
            }
            return;                          // 처리 끝
        }
    }

    /* 2) 프레임이 아니면: 예전 텍스트 프로토콜 하위호환 */
    char received_string[256];
    size_t n = MIN(sizeof(received_string)-1, (size_t)len);
    memcpy(received_string, data, n);        // 그대로 문자열 복사
    received_string[n] = '\0';               // 문자열 종료
    handle_ascii_command(received_string);   // ASCII 명령 처리
}

/* NUS 콜백 구조체 등록 */
static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,               // 수신 시 실행할 콜백
    .sent     = nus_sent_cb,                 // 전송 완료 시 실행할 콜백
};

/* ============================================================
 *             UART 콜백 (개행 의존 제거, 임계치/타임아웃 flush)
 * ============================================================*/
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(dev); ARG_UNUSED(user_data);   // 사용하지 않는 인자는 매크로로 무시

    static struct uart_data_t *rx_buf = NULL; // 현재 UART 수신 데이터를 모으는 버퍼
    static bool disable_req;                  // 수신 중단 요청 플래그
    static uint8_t *aborted_buf;              // 중단된 전송 버퍼 포인터
    static size_t aborted_len;                // 중단된 길이 추적
    struct uart_data_t *tx_buf;               // 송신 버퍼 포인터

    switch (evt->type) {                      // UART 이벤트 종류에 따라 분기
    case UART_TX_DONE:                        // 송신이 끝났을 때
        if (!evt->data.tx.len || !evt->data.tx.buf) return; // 전송 길이나 포인터가 없으면 무시
        tx_buf = aborted_buf
               ? CONTAINER_OF(aborted_buf, struct uart_data_t, data[0]) // 중단된 경우
               : CONTAINER_OF(evt->data.tx.buf, struct uart_data_t, data[0]); // 일반 경우
        k_free(tx_buf);                       // 전송 끝난 버퍼 해제
        aborted_buf = NULL;                   // 상태 초기화
        aborted_len = 0;

        tx_buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT); // FIFO에서 다음 보낼 버퍼 꺼냄
        if (tx_buf) {
            if (uart_tx(uart, tx_buf->data, tx_buf->len, SYS_FOREVER_MS)) {
                LOG_WRN("uart_tx failed on FIFO buf"); // 전송 실패 시 경고
            }
        }
        break;

    case UART_TX_ABORTED:                     // 송신이 중단됐을 때
        if (!aborted_buf) {                   // 처음 중단된 경우
            aborted_buf = (uint8_t *)evt->data.tx.buf;
            aborted_len = 0;
        }
        aborted_len += evt->data.tx.len;      // 중단된 만큼 길이 더하기
        tx_buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data[0]); // 원래 버퍼 찾기
        uart_tx(uart, &tx_buf->data[aborted_len], tx_buf->len - aborted_len, SYS_FOREVER_MS); // 남은 부분 다시 전송
        break;

    case UART_RX_RDY:                         // 수신 데이터가 들어왔을 때
        if (!rx_buf) return;                  // 수신 버퍼 없으면 무시
        {
            size_t space_left = sizeof(rx_buf->data) - rx_buf->len; // 남은 공간 계산
            size_t to_copy = MIN(space_left, evt->data.rx.len);     // 복사 가능한 길이
            if (to_copy == 0) {               // 공간 부족 → 수신 비활성화
                uart_rx_disable(uart);
                return;
            }
            memcpy(&rx_buf->data[rx_buf->len], // 버퍼 끝에 새 데이터 복사
                   &evt->data.rx.buf[evt->data.rx.offset],
                   to_copy);
            rx_buf->len += to_copy;           // 길이 갱신

            if (disable_req) return;          // 이미 중단 요청 중이면 리턴

            /* 임계치 도달 시 끊어서 처리 */
            if (rx_buf->len >= UART_FLUSH_THRESHOLD) {
                disable_req = true;           // 중단 요청 플래그 세팅
                uart_rx_disable(uart);        // 수신 중단 → RX_DISABLED 이벤트 발생 예정
            }
        }
        break;

    case UART_RX_DISABLED:                    // 수신이 꺼졌을 때
        disable_req = false;                  // 중단 요청 해제

        /* 받은 데이터가 있으면 BLE로 프레임 전송 (워크로 처리) */
        if (rx_buf && rx_buf->len > 0) {
            pending_buf = rx_buf;             // 보류 버퍼에 넘겨놓고
            rx_buf = NULL;
            k_work_submit(&ble_send_work);    // BLE 전송 워크 제출
        } else {
            if (rx_buf) k_free(rx_buf);       // 데이터 없으면 그냥 해제
            rx_buf = NULL;
        }

        /* 새 버퍼로 다시 수신 시작 */
        rx_buf = k_malloc(sizeof(*rx_buf));
        
        if (!rx_buf) {                        // 메모리 부족 시
            LOG_WRN("rx malloc failed, reschedule rx enable");
            k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY); // 일정 시간 후 재시도
            return;
        }
        rx_buf->len = 0;
        uart_rx_enable(uart, rx_buf->data, sizeof(rx_buf->data), UART_WAIT_FOR_RX); // 수신 재시작
        break;

    case UART_RX_BUF_REQUEST:                 // 드라이버가 새 RX 버퍼를 요청할 때
        rx_buf = k_malloc(sizeof(*rx_buf));   // 새 버퍼 할당
        if (rx_buf) {
            rx_buf->len = 0;
            uart_rx_buf_rsp(uart, rx_buf->data, sizeof(rx_buf->data)); // 드라이버에 제공
        } else {
            LOG_WRN("malloc failed on BUF_REQUEST"); // 메모리 부족 시 경고
        }
        break;

    case UART_RX_BUF_RELEASED: {              // 드라이버가 버퍼 반환할 때
        struct uart_data_t *released_buf =
            CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data[0]); // 원래 구조체 포인터로 변환
        if (released_buf->len > 0) {
            k_fifo_put(&fifo_uart_rx_data, released_buf); // 데이터 있으면 FIFO에 넣음
        } else {
            k_free(released_buf);             // 없으면 바로 해제
        }
    } break;

    default:                                  // 그 외 이벤트는 무시
        break;
    }
}

/* UART 워크/초기화 */
static void uart_work_handler(struct k_work *item) // RX 버퍼 확보 재시도 워크
{
    struct uart_data_t *buf = k_malloc(sizeof(*buf)); // 새 RX 버퍼 할당
    if (!buf) {
        LOG_WRN("rx alloc failed, reschedule");       // 실패 시 경고
        k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY); // 일정 시간 뒤 재시도
        return;
    }
    buf->len = 0;
    uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX); // UART RX 활성화
}

static bool uart_test_async_api(const struct device *dev) // 드라이버가 async API 지원 여부 확인
{
    const struct uart_driver_api *api =
        (const struct uart_driver_api *)dev->api;     // 드라이버 API 캐스팅
    return (api->callback_set != NULL);               // 콜백 등록 함수가 있으면 async 지원
}

static int uart_init_wrap(void)              // UART 초기화 래퍼 함수
{
    int err;
    struct uart_data_t *rx;

    if (!device_is_ready(uart)) {            // UART 장치 준비 확인
        return -ENODEV;
    }

    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) { // USB CDC ACM을 사용하는 경우
        err = usb_enable(NULL);              // USB 활성화
        if (err && (err != -EALREADY)) {     // 이미 켜져있지 않고 에러면
            LOG_ERR("Failed to enable USB");
            return err;
        }
    }

    rx = k_malloc(sizeof(*rx));              // 첫 수신 버퍼 할당
    if (!rx) return -ENOMEM;
    rx->len = 0;

    k_work_init_delayable(&uart_work, uart_work_handler); // 워크 초기화

    if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
        uart_async_adapter_init(async_adapter, uart); // 드라이버를 async 어댑터로 래핑
        uart = async_adapter;                         // 래핑된 장치 핸들을 사용
    }

    err = uart_callback_set(uart, uart_cb, NULL); // UART 콜백 등록
    if (err) {
        k_free(rx);
        LOG_ERR("Cannot initialize UART callback");
        return err;
    }

    /* 시작 메시지(개발용) */
    struct uart_data_t *tx = k_malloc(sizeof(*tx)); // 시작 시점에 전송할 버퍼
    if (tx) {
        const char *msg = "Starting NUS peripheral (Binary+Frame)\r\n"; // 안내 메시지
        size_t n = MIN(sizeof(tx->data), strlen(msg));
        memcpy(tx->data, msg, n);
        tx->len = (uint16_t)n;
        err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS); // UART로 송신
        if (err) {
            k_fifo_put(&fifo_uart_tx_data, tx);  // 실패 시 FIFO에 넣어둠
        }
    }

    err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX); // 수신 시작
    if (err) {
        LOG_ERR("Cannot enable uart reception (err: %d)", err);
        k_free(rx);
    }
    return err;                                // 초기화 결과 리턴
}

/* ============================================================
 *                        BLE 연결 콜백
 * ============================================================*/
static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];                 // 연결된 장치 주소 문자열 버퍼

    if (err) {                                     // 연결 실패 시
        LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // 연결 상대 주소 문자열 변환
    current_conn = bt_conn_ref(conn);             // 현재 연결 포인터 보관(참조 카운트 증가)
    dk_set_led_on(CON_STATUS_LED);                // 연결 상태 LED ON
    update_mtu_from_conn(conn);                   // 연결된 conn에서 MTU 값 갱신
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // 연결 상대 주소 문자열 변환

    LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

    if (auth_conn) {                              // 인증 연결 핸들이 있으면 해제
        bt_conn_unref(auth_conn);
        auth_conn = NULL;
    }
    if (current_conn) {                           // 현재 연결 핸들이 있으면 해제
        bt_conn_unref(current_conn);
        current_conn = NULL;
        dk_set_led_off(CON_STATUS_LED);           // 연결 상태 LED OFF
    }
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
                             enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    if (!err) {                                   // 보안 수준 변경 성공
        LOG_INF("Security changed: %s level %u", addr, level);
    } else {                                      // 보안 설정 실패
        LOG_WRN("Security failed: %s level %u err %d %s", addr, level, err,
                bt_security_err_to_str(err));
    }
}
#endif

// BLE 연결 이벤트 콜백 등록
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,                    // 연결 성공 시 실행
    .disconnected = disconnected,                 // 연결 끊김 시 실행
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
    .security_changed = security_changed,         // 보안 수준 변경 시 실행
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
/* ========== 보안 관련 콜백 ========== */
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) // 패스키 표시
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey) // 패스키 확인
{
    char addr[BT_ADDR_LE_STR_LEN];
    auth_conn = bt_conn_ref(conn);                // 인증 연결 참조 저장
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Passkey for %s: %06u", addr, passkey);
    LOG_INF("Press Button to confirm/reject.");   // 버튼 입력으로 결정
}

static void auth_cancel(struct bt_conn *conn)     // 페어링 취소
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded) // 페어링 완료
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason) // 페어링 실패
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
            bt_security_err_to_str(reason));
}

// 인증 관련 콜백 등록 구조체
static struct bt_conn_auth_cb conn_auth_callbacks = {
    .passkey_display = auth_passkey_display,
    .passkey_confirm = auth_passkey_confirm,
    .cancel = auth_cancel,
};

// 페어링 완료/실패 콜백 등록 구조체
static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete,
    .pairing_failed   = pairing_failed
};
#else
// 보안 기능을 끄면 빈 콜백 구조체
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

/* ============================================================
 *                       기타 유틸
 * ============================================================*/
static void error(void)                          // 치명적 에러 발생 시 LED 켜고 무한 루프
{
    dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);
    while (true) { k_sleep(K_MSEC(1000)); }
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)          // 숫자 비교 결과 버튼 입력 반영
{
    if (accept) {
        bt_conn_auth_passkey_confirm(auth_conn); // 수락
    } else {
        bt_conn_auth_cancel(auth_conn);          // 거절
    }
    bt_conn_unref(auth_conn);                    // 연결 참조 해제
    auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed) // 버튼 이벤트 콜백
{
    uint32_t buttons = button_state & has_changed; // 이번에 바뀐 버튼만 추출
    if (auth_conn) {
        if (buttons & KEY_PASSKEY_ACCEPT) num_comp_reply(true);  // 수락 버튼
        if (buttons & KEY_PASSKEY_REJECT) num_comp_reply(false); // 거절 버튼
    }
}
#endif

static void configure_gpio(void)                 // GPIO 초기화 (LED/버튼)
{
    int err;
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
    err = dk_buttons_init(button_changed);       // 버튼 이벤트 초기화
    if (err) { LOG_ERR("Cannot init buttons (err: %d)", err); }
#endif
    err = dk_leds_init();                        // LED 제어 초기화
    if (err) { LOG_ERR("Cannot init LEDs (err: %d)", err); }
}

/* ============================================================
 *              BLE로 보낼 워크 핸들러 (UART 수신 → 프레임)
 * ============================================================*/
static void ble_send_work_handler(struct k_work *work)
{
    if (!current_conn || !pending_buf) {         // 연결 없거나 전송할 버퍼 없으면 skip
        printk("[PERIPHERAL] No BLE conn or buffer, skip\n");
        return;
    }

    /* pending_buf 내용을 payload로 하여 프레임 전송 */
    struct uart_data_t *copy = pending_buf;
    pending_buf = NULL;

    int err = send_frame_over_nus(copy->data, copy->len, DEFAULT_CH_MASK, 0x00);
    if (err) {
        printk("[PERIPHERAL] BLE send failed (err: %d)\n", err);
    }
    k_free(copy);                                // 전송 후 메모리 해제
}

/* ============================================================
 *                          main()
 * ============================================================*/
int main(void)
{
    int blink_status = 0;                        // LED 깜박 상태 토글 변수
    int err = 0;

    configure_gpio();                            // LED/버튼 초기화
    k_work_init(&ble_send_work, ble_send_work_handler); // BLE 전송 워크 초기화

    err = uart_init_wrap();                      // UART 초기화
    if (err) {
        error();                                 // 실패 시 에러 핸들러 진입
    }

    /* UART 시작 메세지(개발용) */
    {
        struct uart_data_t *tx = k_malloc(sizeof(*tx)); // TX 버퍼 준비
        if (tx) {
            snprintf((char*)tx->data, sizeof(tx->data), "Test startup TX\n");
            tx->len = (uint16_t)strlen((char*)tx->data);
            uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS); // 시작 메시지 전송
            /* free는 TX_DONE에서 */
        }
    }

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
    err = bt_conn_auth_cb_register(&conn_auth_callbacks); // 인증 콜백 등록
    if (err) {
        printk("Failed to register authorization callbacks.\n");
        return 0;
    }
    err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks); // 페어링 콜백 등록
    if (err) {
        printk("Failed to register authorization info callbacks.\n");
        return 0;
    }
#endif

    printk("4-channel ADC print to UART start\n");

    if (!device_is_ready(adc_dev)) {             // ADC 장치 준비 확인
        printk("ADC device not ready!\n");
        return 0;
    }
    /* 개별 채널 setup은 adc_read_4ch에서 수행 */

    err = bt_enable(NULL);                       // BLE 스택 활성화
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }
    LOG_INF("Bluetooth initialized");

    k_sem_give(&ble_init_ok);                    // BLE 초기화 완료 신호 주기

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();                         // 플래시에서 settings 로드
    }

    err = bt_nus_init(&nus_cb);                  // NUS 서비스 초기화
    if (err) {
        LOG_ERR("Failed to initialize UART service (err: %d)", err);
        return 0;
    }

    // 광고 시작 (연결 가능 모드, ad/sd 데이터 포함)
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return 0;
    }

    // 메인 루프: LED 깜박이며 대기 (주기 1초)
    for (;;) {
        dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
        k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
    }
    return 0;
}

