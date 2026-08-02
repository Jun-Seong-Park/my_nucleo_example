/**
 * Step 1+3 — LED blink + USART1 로그 송신 (HAL 미사용, CMSIS 레지스터 직접 제어)
 *
 * 대상 보드 : NUCLEO-F103RB (STM32F103RBT6)
 * 클럭      : 리셋 기본값 HSI 8 MHz (SystemInit 이 클럭을 건드리지 않음)
 *             → SYSCLK = AHB = APB1 = APB2 = 8 MHz
 *
 * 핀 배치
 *   PA6  : 외부 LED (빵판, Arduino 헤더의 MISO/D12 자리)
 *   PA9  : USART1_TX  → USB-UART 어댑터의 RX
 *   PA10 : USART1_RX  ← USB-UART 어댑터의 TX
 *
 * 참고 문서 : RM0008 Rev 21
 *             - 8.2.2  GPIOx_CRL / 8.2.3 GPIOx_CRH
 *             - 8.2.5  GPIOx_BSRR
 *             - 7.3.7  RCC_APB2ENR
 *             - 27.6   USART registers
 */

#include "main.h"

/* ------------------------------------------------------------------ */
/* 보드 / 통신 설정                                                    */
/* ------------------------------------------------------------------ */

#define LED_PORT   GPIOA
#define LED_PIN    6u          /* PA6 = 빵판의 외부 LED */

#define PCLK2_HZ   8000000u    /* USART1 은 APB2 에 붙어 있다 */
#define UART_BAUD  115200u

/* ------------------------------------------------------------------ */
/* GPIO — LED                                                          */
/* ------------------------------------------------------------------ */

/**
 * PA6 를 푸시풀 출력으로 설정한다.
 *
 * STM32F1 의 GPIO 설정은 핀 하나당 4비트다.
 *   CRL : 핀 0 ~ 7   (핀 n → 비트 [4n+3 : 4n])
 *   CRH : 핀 8 ~ 15  (핀 n → 비트 [4(n-8)+3 : 4(n-8)])
 *
 * 4비트 필드 = [ CNF(2bit) | MODE(2bit) ]
 *   MODE = 00 입력 / 01 출력10MHz / 10 출력2MHz / 11 출력50MHz
 *   CNF(출력) = 00 범용푸시풀 / 01 범용오픈드레인 / 10 대체기능푸시풀 / 11 대체기능오픈드레인
 *   CNF(입력) = 00 아날로그   / 01 플로팅       / 10 풀업풀다운
 *
 * LED 는 빠를 필요가 없으므로 MODE=10, CNF=00 → 0b0010 = 0x2.
 */
static void led_init(void)
{
    /* 1) GPIOA 에 클럭을 공급한다. 클럭이 없으면 레지스터에 써도 값이 남지 않는다. */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    /* 2) CRL 의 PA6 자리(비트 27:24)만 0x2 로 바꾼다. */
    const uint32_t shift = LED_PIN * 4u;
    const uint32_t mask  = 0xFu << shift;
    const uint32_t value = 0x2u << shift;

    uint32_t crl = LED_PORT->CRL;
    crl &= ~mask;
    crl |= value;
    LED_PORT->CRL = crl;
}

/** BSRR 하위 16비트(BSx)에 1 → 해당 핀 SET. 한 번의 쓰기로 원자적이다. */
static void led_on(void)
{
    LED_PORT->BSRR = (1u << LED_PIN);
}

/** BSRR 상위 16비트(BRx)에 1 → 해당 핀 RESET. */
static void led_off(void)
{
    LED_PORT->BSRR = (1u << (LED_PIN + 16u));
}

/* ------------------------------------------------------------------ */
/* USART1 — 115200 8N1                                                 */
/* ------------------------------------------------------------------ */

/**
 * USART1 을 115200 8N1 로 초기화한다.
 *
 * BRR 계산 (오버샘플링 16):
 *   USARTDIV = PCLK2 / (16 x baud) = 8000000 / (16 x 115200) = 4.3403
 *   BRR = [ DIV_Mantissa(12bit) | DIV_Fraction(4bit) ]
 *       = (4 << 4) | round(0.3403 x 16) = (4 << 4) | 5 = 0x45 = 69
 *
 *   정수 나눗셈 한 번으로 같은 값이 나온다 : 8000000 / 115200 = 69
 *   실제 통신속도 = 8000000 / 69 = 115942 baud → 오차 +0.64% (허용범위 내)
 */
static void uart1_init(void)
{
    /* 1) GPIOA 와 USART1 클럭. 둘 다 APB2 에 있다. */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    /* 2) PA9 = 대체기능 푸시풀 출력, PA10 = 플로팅 입력.
     *    핀 9  → CRH 비트 [7:4]   : CNF=10, MODE=11 → 0b1011 = 0xB
     *    핀 10 → CRH 비트 [11:8]  : CNF=01, MODE=00 → 0b0100 = 0x4
     *
     *    PA9/PA10 은 USART1 의 기본 매핑이라 AFIO 리맵이 필요 없다.
     *    (리맵을 쓸 때만 RCC_APB2ENR_AFIOEN 이 필요하다) */
    uint32_t crh = GPIOA->CRH;
    crh &= ~((0xFu << 4) | (0xFu << 8));
    crh |=  (0xBu << 4);
    crh |=  (0x4u << 8);
    GPIOA->CRH = crh;

    /* 3) 통신 속도 */
    USART1->BRR = PCLK2_HZ / UART_BAUD;

    /* 4) 1 스톱비트 / 흐름제어 없음 (리셋값과 같지만 명시해 둔다) */
    USART1->CR2 = 0u;
    USART1->CR3 = 0u;

    /* 5) 워드길이 8bit(M=0), 패리티 없음(PCE=0) 상태에서
     *    송신부·수신부·USART 본체를 켠다. */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/** 한 바이트 송신. TXE(송신 데이터 레지스터 비었음)를 기다린 뒤 DR 에 쓴다. */
static void uart1_put_char(char c)
{
    while ((USART1->SR & USART_SR_TXE) == 0u)
    {
        /* 이전 바이트가 시프트 레지스터로 옮겨질 때까지 대기 */
    }
    USART1->DR = (uint8_t)c;
}

/** 널 종료 문자열 송신. */
static void uart1_put_str(const char *s)
{
    while (*s != '\0')
    {
        uart1_put_char(*s);
        s++;
    }
}

/** 부호 없는 10진수 송신. printf 를 쓰지 않으려고 직접 만든다. */
static void uart1_put_u32(uint32_t v)
{
    char digits[10];
    uint32_t i = 0u;

    if (v == 0u)
    {
        uart1_put_char('0');
        return;
    }

    /* 뒤에서부터 한 자리씩 뽑아 담고 */
    while (v > 0u)
    {
        digits[i] = (char)('0' + (v % 10u));
        v /= 10u;
        i++;
    }

    /* 역순으로 내보낸다 */
    while (i > 0u)
    {
        i--;
        uart1_put_char(digits[i]);
    }
}

/* ------------------------------------------------------------------ */
/* 로그 / 지연                                                         */
/* ------------------------------------------------------------------ */

/**
 * 현재 LED 상태를 한 줄로 보낸다.
 *
 * 마지막 항목은 코드가 기억하는 값이 아니라
 * ODR 레지스터를 실제로 읽어온 핀 출력 상태다.
 */
static void log_led_state(uint32_t count, const char *state)
{
    const uint32_t odr_bit = (LED_PORT->ODR >> LED_PIN) & 1u;

    uart1_put_str("[");
    uart1_put_u32(count);
    uart1_put_str("] LED ");
    uart1_put_str(state);
    uart1_put_str("  PA6=");
    uart1_put_char((char)('0' + odr_bit));
    uart1_put_str("\r\n");
}

/**
 * 대충 기다리면서, 그 사이에 들어온 문자가 있으면 그대로 되돌려 보낸다(에코).
 *
 * volatile 을 붙여야 컴파일러가 빈 루프를 지우지 않는다.
 * 시간이 정확하지 않다 — Step 2 에서 SysTick 으로 교체할 예정.
 */
static void delay_with_echo(volatile uint32_t count)
{
    while (count > 0u)
    {
        /* RXNE = 수신 데이터 레지스터에 읽을 것이 있음.
         * DR 을 읽으면 RXNE 는 하드웨어가 자동으로 지운다. */
        if ((USART1->SR & USART_SR_RXNE) != 0u)
        {
            const char received = (char)(USART1->DR & 0xFFu);
            /* 터미널 인코딩에 휘둘리지 않도록 보내는 문자열은 ASCII 로만 쓴다 */
            uart1_put_str("\r\n  <- RX: '");
            uart1_put_char(received);
            uart1_put_str("'\r\n");
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

    uart1_put_str("\r\n");
    uart1_put_str("========================================\r\n");
    uart1_put_str(" STM32F103RB blink logger\r\n");
    uart1_put_str(" no HAL / direct register access\r\n");
    uart1_put_str(" USART1 PA9(TX) PA10(RX) 115200 8N1\r\n");
    uart1_put_str("========================================\r\n");

    uint32_t count = 0u;

    while (1)
    {
        count++;

        led_on();
        log_led_state(count, "ON ");
        delay_with_echo(200000u);

        led_off();
        log_led_state(count, "OFF");
        delay_with_echo(200000u);
    }
}
