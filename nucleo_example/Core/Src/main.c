/**
 * LED blink + USART1 로그 + 조이스틱 관측 + 택트 스위치 EXTI
 * (HAL 미사용, CMSIS 레지스터 직접 제어)
 *
 * 대상 보드 : NUCLEO-F103RB (STM32F103RBT6)
 * 클럭      : 리셋 기본값 HSI 8 MHz (SystemInit 이 클럭을 건드리지 않음)
 *             → SYSCLK = AHB = APB1 = APB2 = 8 MHz, ADC 클럭 = PCLK2/2 = 4 MHz
 *
 * 역할 구분
 *   택트 스위치 : LED 를 제어한다. 누르고 있는 동안 blink 정지.
 *   조이스틱    : 아무것도 제어하지 않는다. VRx/VRy/SW 를 시리얼로 보여주기만 한다.
 *
 * 핀 배치 (괄호는 보드 앞면 Arduino 실크)
 *   PA6  (D12) : 외부 LED
 *   PA9  (D8)  : USART1_TX  → USB-UART 어댑터 RX
 *   PA10 (D2)  : USART1_RX  ← USB-UART 어댑터 TX
 *   PA0  (A0)  : 조이스틱 VRx → ADC1_IN0   (표시 전용)
 *   PA1  (A1)  : 조이스틱 VRy → ADC1_IN1   (표시 전용)
 *   PA4  (A2)  : 조이스틱 SW  → 풀업 입력  (표시 전용, 인터럽트 안 씀)
 *   PB0  (A3)  : 택트 스위치  → EXTI0 양쪽 엣지
 *
 * 참고 문서 : RM0008 Rev 21
 *             - 8.2.2/8.2.3 GPIOx_CRL, GPIOx_CRH      - 9.3 EXTI
 *             - 11.3.1 ADC 캘리브레이션                - 27.6 USART registers
 */

#include "main.h"

/* ------------------------------------------------------------------ */
/* 보드 / 통신 설정                                                    */
/* ------------------------------------------------------------------ */

#define LED_PORT    GPIOA
#define LED_PIN     6u          /* PA6 */

#define TACT_PORT   GPIOB
#define TACT_PIN    0u          /* PB0 = Arduino A3 → EXTI0 */

#define JOYSW_PORT  GPIOA
#define JOYSW_PIN   4u          /* PA4 = Arduino A2, 읽기만 한다 */

#define ADC_CH_VRX  0u          /* PA0 = ADC1_IN0 */
#define ADC_CH_VRY  1u          /* PA1 = ADC1_IN1 */

#define PCLK2_HZ    8000000u    /* USART1 은 APB2 에 붙어 있다 */
#define UART_BAUD   115200u

/* ------------------------------------------------------------------ */
/* 인터럽트와 메인 루프가 주고받는 상태                                */
/*                                                                     */
/* ISR 은 이 두 변수만 건드리고 즉시 빠져나온다. UART 송신 같은        */
/* 오래 걸리는 일은 절대 ISR 안에서 하지 않는다 — 그래야 송신 루틴이   */
/* 인터럽트 때문에 끊기지 않는다.                                      */
/* volatile 이 없으면 컴파일러가 "메인 루프에서 안 바뀌는 값"으로 보고 */
/* 레지스터에 캐싱해버려서 ISR 의 변경을 영영 못 본다.                 */
/* ------------------------------------------------------------------ */

static volatile uint32_t g_tact_held = 0u;      /* 1 = 택트를 누르고 있는 중 */
static volatile uint32_t g_tact_changed = 0u;   /* 1 = 상태가 막 바뀜        */

/* ------------------------------------------------------------------ */
/* 공용 : 핀 하나의 CRL/CRH 4비트 필드를 바꾼다                        */
/* ------------------------------------------------------------------ */

/**
 * STM32F1 의 GPIO 설정은 핀 하나당 4비트다.
 *   CRL : 핀 0~7   (핀 n → 비트 [4n+3 : 4n])
 *   CRH : 핀 8~15  (핀 n → 비트 [4(n-8)+3 : 4(n-8)])
 *
 * 4비트 필드 = [ CNF(2bit) | MODE(2bit) ]
 *   MODE = 00 입력 / 01 출력10MHz / 10 출력2MHz / 11 출력50MHz
 *   CNF(출력) = 00 범용푸시풀 / 01 범용오픈드레인 / 10 AF푸시풀 / 11 AF오픈드레인
 *   CNF(입력) = 00 아날로그   / 01 플로팅        / 10 풀업풀다운
 */
static void gpio_set_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t cfg4)
{
    volatile uint32_t *cr = (pin < 8u) ? &port->CRL : &port->CRH;
    const uint32_t shift = (pin % 8u) * 4u;

    uint32_t v = *cr;
    v &= ~(0xFu << shift);
    v |=  (cfg4 << shift);
    *cr = v;
}

/* ------------------------------------------------------------------ */
/* GPIO — LED                                                          */
/* ------------------------------------------------------------------ */

static void led_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    gpio_set_mode(LED_PORT, LED_PIN, 0x2u);   /* CNF=00 푸시풀, MODE=10 (2MHz) */
}

/** BSRR 하위 16비트(BSx)에 1 → 핀 SET. 한 번의 쓰기로 원자적이다. */
static void led_on(void)
{
    LED_PORT->BSRR = (1u << LED_PIN);
}

/** BSRR 상위 16비트(BRx)에 1 → 핀 RESET. */
static void led_off(void)
{
    LED_PORT->BSRR = (1u << (LED_PIN + 16u));
}

/* ------------------------------------------------------------------ */
/* USART1 — 115200 8N1                                                 */
/* ------------------------------------------------------------------ */

/**
 * BRR 계산 (오버샘플링 16):
 *   USARTDIV = 8000000 / (16 x 115200) = 4.3403
 *   BRR = [DIV_Mantissa(12bit) | DIV_Fraction(4bit)] = (4<<4)|5 = 0x45 = 69
 *   정수 나눗셈 한 번으로 같은 값 : 8000000 / 115200 = 69
 *   실제 속도 8000000/69 = 115942 baud → 오차 +0.64% (허용범위 내)
 */
static void uart1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    /* PA9  : CNF=10(AF 푸시풀), MODE=11(50MHz) → 0xB
     * PA10 : CNF=01(플로팅),    MODE=00(입력)  → 0x4
     * PA9/PA10 은 USART1 기본 매핑이라 AFIO 리맵이 필요 없다. */
    gpio_set_mode(GPIOA, 9u,  0xBu);
    gpio_set_mode(GPIOA, 10u, 0x4u);

    USART1->BRR = PCLK2_HZ / UART_BAUD;
    USART1->CR2 = 0u;                                          /* 1 스톱비트    */
    USART1->CR3 = 0u;                                          /* 흐름제어 없음 */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;   /* 8N1, 송수신on */
}

/** 한 바이트 송신. TXE(송신 레지스터 비었음)를 기다린 뒤 DR 에 쓴다. */
static void uart1_put_char(char c)
{
    while ((USART1->SR & USART_SR_TXE) == 0u)
    {
    }
    USART1->DR = (uint8_t)c;
}

static void uart1_put_str(const char *s)
{
    while (*s != '\0')
    {
        uart1_put_char(*s);
        s++;
    }
}

/** 부호 없는 10진수 송신. width 만큼 앞을 공백으로 채운다(0 이면 채우지 않음). */
static void uart1_put_u32(uint32_t v, uint32_t width)
{
    char digits[10];
    uint32_t n = 0u;

    do
    {
        digits[n] = (char)('0' + (v % 10u));
        v /= 10u;
        n++;
    } while (v > 0u);

    while (width > n)
    {
        uart1_put_char(' ');
        width--;
    }

    while (n > 0u)
    {
        n--;
        uart1_put_char(digits[n]);
    }
}

/* ------------------------------------------------------------------ */
/* ADC1 — 조이스틱 VRx / VRy (관측 전용)                               */
/* ------------------------------------------------------------------ */

/**
 * ADC1 을 12비트 단일 변환 모드로 준비한다.
 *
 * ADC 클럭은 PCLK2 를 RCC_CFGR 의 ADCPRE 로 나눈 값이고, 리셋 기본값이
 * /2 라서 8MHz/2 = 4MHz 다. F1 의 상한 14MHz 이내이므로 그대로 쓴다.
 *
 * ★ F1 고유 : 전원을 켠 뒤 반드시 캘리브레이션을 해야 한다.
 *   빼먹으면 변환값이 수십 LSB 씩 치우친다. F4 이후 계열엔 없는 절차다.
 */
static void adc1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    /* PA0, PA1 = 아날로그 입력 : CNF=00, MODE=00 → 0x0 */
    gpio_set_mode(GPIOA, 0u, 0x0u);
    gpio_set_mode(GPIOA, 1u, 0x0u);

    /* 샘플링 시간을 채널0,1 모두 239.5 사이클(0b111)로 최대한 길게.
     * 조이스틱 포텐쇼미터는 출력 임피던스가 높아서 짧게 잡으면 값이 흔들린다.
     * SMPR2 : SMP0=[2:0], SMP1=[5:3] */
    ADC1->SMPR2 |= (0x7u << 0) | (0x7u << 3);

    /* 시퀀스 길이 1 (SQR1_L=0) — 한 번에 채널 하나만 변환 */
    ADC1->SQR1 = 0u;

    /* 전원 인가. 꺼져 있을 때의 첫 ADON 은 깨우기만 하고 변환을 시작하지 않는다. */
    ADC1->CR2 = ADC_CR2_ADON;
    for (volatile uint32_t i = 0u; i < 2000u; i++)
    {
        /* tSTAB 안정화 대기 */
    }

    /* 캘리브레이션 레지스터 리셋 → 캘리브레이션 실행.
     * 두 비트 모두 완료되면 하드웨어가 스스로 0 으로 내린다. */
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0u)
    {
    }

    ADC1->CR2 |= ADC_CR2_CAL;
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0u)
    {
    }
}

/** 채널 하나를 변환해서 0~4095 를 돌려준다. */
static uint16_t adc1_read(uint32_t channel)
{
    ADC1->SQR3 = channel;          /* 변환할 채널 지정 (시퀀스 1번 자리) */
    ADC1->CR2 |= ADC_CR2_ADON;     /* ADON=1 인 상태에서 또 1 을 쓰면 변환 시작 */

    while ((ADC1->SR & ADC_SR_EOC) == 0u)
    {
    }

    return (uint16_t)(ADC1->DR & 0x0FFFu);   /* DR 을 읽으면 EOC 가 지워진다 */
}

/* ------------------------------------------------------------------ */
/* 입력 핀들                                                           */
/* ------------------------------------------------------------------ */

/**
 * 택트 스위치(PB0) 와 조이스틱 SW(PA4) 를 모두 내부 풀업 입력으로 만든다.
 * 두 스위치 모두 "한쪽은 핀, 반대쪽은 GND" 인 단순 구조라 외부 저항이 필요 없다.
 * 평소 HIGH, 누르면 LOW 가 된다.
 *
 * 차이는 감시 방식이다.
 *   택트    : EXTI0 인터럽트로 즉시 반응한다 (LED 를 제어하므로).
 *   조이스틱: 인터럽트를 걸지 않는다. 로그를 찍을 때 IDR 을 한 번 읽을 뿐이다.
 *
 * EXTIn 라인은 포트를 골라서 물린다. AFIO_EXTICR 이 그 선택기이고,
 * 이 레지스터를 건드리려면 AFIO 클럭이 켜져 있어야 한다.
 *   EXTICR[0] → EXTI0..3,  필드값 0000=PA, 0001=PB, 0010=PC
 */
static void inputs_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    /* --- 조이스틱 SW : PA4, 읽기 전용 --- */
    gpio_set_mode(JOYSW_PORT, JOYSW_PIN, 0x8u);   /* CNF=10 풀업풀다운, MODE=00 입력 */
    JOYSW_PORT->BSRR = (1u << JOYSW_PIN);         /* ODR=1 → 풀업 선택 */

    /* --- 택트 스위치 : PB0, EXTI0 --- */
    gpio_set_mode(TACT_PORT, TACT_PIN, 0x8u);
    TACT_PORT->BSRR = (1u << TACT_PIN);

    AFIO->EXTICR[0] &= ~(0xFu << 0);
    AFIO->EXTICR[0] |=  (0x1u << 0);              /* EXTI0 ← 포트 B */

    EXTI->FTSR |= (1u << TACT_PIN);               /* 누를 때 (HIGH→LOW) */
    EXTI->RTSR |= (1u << TACT_PIN);               /* 뗄 때   (LOW→HIGH) */
    EXTI->IMR  |= (1u << TACT_PIN);

    NVIC_EnableIRQ(EXTI0_IRQn);
}

/** 조이스틱 SW 가 눌려 있으면 1. 풀업이므로 눌린 상태가 LOW 다. */
static uint32_t joysw_pressed(void)
{
    return (((JOYSW_PORT->IDR >> JOYSW_PIN) & 1u) == 0u) ? 1u : 0u;
}

/**
 * 택트 스위치 (PB0) — 누르고 있는 동안만 blink 정지.
 *
 * 양쪽 엣지를 다 받아서 현재 핀 상태를 그대로 반영한다.
 * 채터링으로 여러 번 들어와도 값이 실제로 바뀔 때만 메인 루프에 알린다.
 */
void EXTI0_IRQHandler(void)
{
    if ((EXTI->PR & (1u << TACT_PIN)) == 0u)
    {
        return;
    }
    EXTI->PR = (1u << TACT_PIN);    /* PR 은 1 을 써서 지운다. 안 지우면 무한 반복 */

    const uint32_t pressed = (((TACT_PORT->IDR >> TACT_PIN) & 1u) == 0u) ? 1u : 0u;

    if (pressed != g_tact_held)
    {
        g_tact_held = pressed;
        g_tact_changed = 1u;
    }
}

/* ------------------------------------------------------------------ */
/* 로그 / 지연                                                         */
/* ------------------------------------------------------------------ */

/** 한 줄 로그 : LED 상태 + 실제 출력(ODR) + 조이스틱 두 축 + 조이스틱 SW. */
static void log_line(uint32_t count, const char *state)
{
    const uint16_t vrx = adc1_read(ADC_CH_VRX);
    const uint16_t vry = adc1_read(ADC_CH_VRY);
    const uint32_t odr_bit = (LED_PORT->ODR >> LED_PIN) & 1u;

    uart1_put_str("[");
    uart1_put_u32(count, 5u);
    uart1_put_str("] ");
    uart1_put_str(state);
    uart1_put_str("  PA6=");
    uart1_put_char((char)('0' + odr_bit));
    uart1_put_str("   VRx=");
    uart1_put_u32(vrx, 4u);
    uart1_put_str("  VRy=");
    uart1_put_u32(vry, 4u);
    uart1_put_str("  SW=");
    uart1_put_str((joysw_pressed() != 0u) ? "DOWN" : "up  ");
    uart1_put_str("\r\n");
}

/** 택트 스위치 상태가 바뀌었으면 한 번만 알린다. */
static void report_tact_change(void)
{
    if (g_tact_changed == 0u)
    {
        return;
    }
    g_tact_changed = 0u;

    if (g_tact_held != 0u)
    {
        uart1_put_str(">>>>> LED PAUSED  (tact held)\r\n");
    }
    else
    {
        uart1_put_str(">>>>> LED RESUMED\r\n");
    }
}

/**
 * 대충 기다리면서 들어온 문자를 에코한다.
 * 택트가 눌리면 즉시 빠져나와 blink 가 바로 멈추게 한다.
 *
 * volatile 을 붙여야 컴파일러가 빈 루프를 지우지 않는다.
 * 시간이 정확하지 않다 — 나중에 SysTick 으로 교체할 예정.
 */
static void delay_tick(volatile uint32_t count, uint32_t abort_on_pause)
{
    while (count > 0u)
    {
        if ((USART1->SR & USART_SR_RXNE) != 0u)
        {
            const char received = (char)(USART1->DR & 0xFFu);
            uart1_put_str("\r\n  <- RX: '");
            uart1_put_char(received);
            uart1_put_str("'\r\n");
        }

        if ((abort_on_pause != 0u) && (g_tact_held != 0u))
        {
            return;
        }

        count--;
    }
}

/* ------------------------------------------------------------------ */
/* 진입점                                                              */
/* ------------------------------------------------------------------ */

int main(void)
{
    led_init();
    uart1_init();
    adc1_init();
    inputs_init();

    uart1_put_str("\r\n");
    uart1_put_str("==============================================\r\n");
    uart1_put_str(" STM32F103RB   no HAL / direct registers\r\n");
    uart1_put_str(" LED  PA6(D12)      UART1 PA9/PA10 115200\r\n");
    uart1_put_str(" JOY  VRx PA0(A0)  VRy PA1(A1)  SW PA4(A2)\r\n");
    uart1_put_str("      -> 표시 전용, LED 에 관여하지 않음\r\n");
    uart1_put_str(" TACT PB0(A3)  hold to pause blink\r\n");
    uart1_put_str("==============================================\r\n");

    uint32_t count = 0u;

    while (1)
    {
        report_tact_change();

        /* 택트를 누르고 있는 동안 : LED 는 끈 채로 조이스틱 값만 계속 보고한다. */
        if (g_tact_held != 0u)
        {
            led_off();
            log_line(count, "PAUSED");
            delay_tick(150000u, 0u);
            continue;
        }

        count++;

        led_on();
        log_line(count, "ON    ");
        delay_tick(200000u, 1u);

        if (g_tact_held != 0u)
        {
            continue;
        }

        led_off();
        log_line(count, "OFF   ");
        delay_tick(200000u, 1u);
    }
}
