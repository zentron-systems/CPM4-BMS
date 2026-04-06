//digital_io.h created Tues 11/02/2026 at 17:50


#ifndef DIGITAL_IO_H
#define DIGITAL_IO_H

#include "main.h"
#include <stdint.h>

/* -------------------------------------------------
 * Types
 * ------------------------------------------------- */

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
}
dpin_t;

typedef enum
{
    LOW  = 0,
    HIGH = 1
}
DigitalLevel;

/* -------------------------------------------------
 * Helpers
 * ------------------------------------------------- */

/* C and C++ friendly pin literal:
 *  - In C (C99+):  ((dpin_t){ PORT, (uint16_t)(1u << N) })
 *  - In C++ (C++11+): dpin_t{ PORT, static_cast<uint16_t>(1u << N) }
 */
#ifdef __cplusplus
  #define PIN_DEF(__PORT, __N)   dpin_t{ (__PORT), static_cast<uint16_t>(1u << (__N)) }
#else
  #define PIN_DEF(__PORT, __N)   ((dpin_t){ (__PORT), (uint16_t)(1u << (__N)) })
#endif

/* -------------------------------------------------
 * Port pin constants (Px0..Px15), emitted only if the port exists
 * Extend through commonly seen ports across STM32 families: A..K and Z
 * ------------------------------------------------- */

#ifdef GPIOA
#define PA0   PIN_DEF(GPIOA, 0)
#define PA1   PIN_DEF(GPIOA, 1)
#define PA2   PIN_DEF(GPIOA, 2)
#define PA3   PIN_DEF(GPIOA, 3)
#define PA4   PIN_DEF(GPIOA, 4)
#define PA5   PIN_DEF(GPIOA, 5)
#define PA6   PIN_DEF(GPIOA, 6)
#define PA7   PIN_DEF(GPIOA, 7)
#define PA8   PIN_DEF(GPIOA, 8)
#define PA9   PIN_DEF(GPIOA, 9)
#define PA10  PIN_DEF(GPIOA, 10)
#define PA11  PIN_DEF(GPIOA, 11)
#define PA12  PIN_DEF(GPIOA, 12)
#define PA13  PIN_DEF(GPIOA, 13)
#define PA14  PIN_DEF(GPIOA, 14)
#define PA15  PIN_DEF(GPIOA, 15)
#endif

#ifdef GPIOB
#define PB0   PIN_DEF(GPIOB, 0)
#define PB1   PIN_DEF(GPIOB, 1)
#define PB2   PIN_DEF(GPIOB, 2)
#define PB3   PIN_DEF(GPIOB, 3)
#define PB4   PIN_DEF(GPIOB, 4)
#define PB5   PIN_DEF(GPIOB, 5)
#define PB6   PIN_DEF(GPIOB, 6)
#define PB7   PIN_DEF(GPIOB, 7)
#define PB8   PIN_DEF(GPIOB, 8)
#define PB9   PIN_DEF(GPIOB, 9)
#define PB10  PIN_DEF(GPIOB, 10)
#define PB11  PIN_DEF(GPIOB, 11)
#define PB12  PIN_DEF(GPIOB, 12)
#define PB13  PIN_DEF(GPIOB, 13)
#define PB14  PIN_DEF(GPIOB, 14)
#define PB15  PIN_DEF(GPIOB, 15)
#endif

#ifdef GPIOC
#define PC0   PIN_DEF(GPIOC, 0)
#define PC1   PIN_DEF(GPIOC, 1)
#define PC2   PIN_DEF(GPIOC, 2)
#define PC3   PIN_DEF(GPIOC, 3)
#define PC4   PIN_DEF(GPIOC, 4)
#define PC5   PIN_DEF(GPIOC, 5)
#define PC6   PIN_DEF(GPIOC, 6)
#define PC7   PIN_DEF(GPIOC, 7)
#define PC8   PIN_DEF(GPIOC, 8)
#define PC9   PIN_DEF(GPIOC, 9)
#define PC10  PIN_DEF(GPIOC, 10)
#define PC11  PIN_DEF(GPIOC, 11)
#define PC12  PIN_DEF(GPIOC, 12)
#define PC13  PIN_DEF(GPIOC, 13)
#define PC14  PIN_DEF(GPIOC, 14)
#define PC15  PIN_DEF(GPIOC, 15)
#endif

#ifdef GPIOD
#define PD0   PIN_DEF(GPIOD, 0)
#define PD1   PIN_DEF(GPIOD, 1)
#define PD2   PIN_DEF(GPIOD, 2)
#define PD3   PIN_DEF(GPIOD, 3)
#define PD4   PIN_DEF(GPIOD, 4)
#define PD5   PIN_DEF(GPIOD, 5)
#define PD6   PIN_DEF(GPIOD, 6)
#define PD7   PIN_DEF(GPIOD, 7)
#define PD8   PIN_DEF(GPIOD, 8)
#define PD9   PIN_DEF(GPIOD, 9)
#define PD10  PIN_DEF(GPIOD, 10)
#define PD11  PIN_DEF(GPIOD, 11)
#define PD12  PIN_DEF(GPIOD, 12)
#define PD13  PIN_DEF(GPIOD, 13)
#define PD14  PIN_DEF(GPIOD, 14)
#define PD15  PIN_DEF(GPIOD, 15)
#endif

#ifdef GPIOE
#define PE0   PIN_DEF(GPIOE, 0)
#define PE1   PIN_DEF(GPIOE, 1)
#define PE2   PIN_DEF(GPIOE, 2)
#define PE3   PIN_DEF(GPIOE, 3)
#define PE4   PIN_DEF(GPIOE, 4)
#define PE5   PIN_DEF(GPIOE, 5)
#define PE6   PIN_DEF(GPIOE, 6)
#define PE7   PIN_DEF(GPIOE, 7)
#define PE8   PIN_DEF(GPIOE, 8)
#define PE9   PIN_DEF(GPIOE, 9)
#define PE10  PIN_DEF(GPIOE, 10)
#define PE11  PIN_DEF(GPIOE, 11)
#define PE12  PIN_DEF(GPIOE, 12)
#define PE13  PIN_DEF(GPIOE, 13)
#define PE14  PIN_DEF(GPIOE, 14)
#define PE15  PIN_DEF(GPIOE, 15)
#endif

#ifdef GPIOF
#define PF0   PIN_DEF(GPIOF, 0)
#define PF1   PIN_DEF(GPIOF, 1)
#define PF2   PIN_DEF(GPIOF, 2)
#define PF3   PIN_DEF(GPIOF, 3)
#define PF4   PIN_DEF(GPIOF, 4)
#define PF5   PIN_DEF(GPIOF, 5)
#define PF6   PIN_DEF(GPIOF, 6)
#define PF7   PIN_DEF(GPIOF, 7)
#define PF8   PIN_DEF(GPIOF, 8)
#define PF9   PIN_DEF(GPIOF, 9)
#define PF10  PIN_DEF(GPIOF, 10)
#define PF11  PIN_DEF(GPIOF, 11)
#define PF12  PIN_DEF(GPIOF, 12)
#define PF13  PIN_DEF(GPIOF, 13)
#define PF14  PIN_DEF(GPIOF, 14)
#define PF15  PIN_DEF(GPIOF, 15)
#endif

#ifdef GPIOG
#define PG0   PIN_DEF(GPIOG, 0)
#define PG1   PIN_DEF(GPIOG, 1)
#define PG2   PIN_DEF(GPIOG, 2)
#define PG3   PIN_DEF(GPIOG, 3)
#define PG4   PIN_DEF(GPIOG, 4)
#define PG5   PIN_DEF(GPIOG, 5)
#define PG6   PIN_DEF(GPIOG, 6)
#define PG7   PIN_DEF(GPIOG, 7)
#define PG8   PIN_DEF(GPIOG, 8)
#define PG9   PIN_DEF(GPIOG, 9)
#define PG10  PIN_DEF(GPIOG, 10)
#define PG11  PIN_DEF(GPIOG, 11)
#define PG12  PIN_DEF(GPIOG, 12)
#define PG13  PIN_DEF(GPIOG, 13)
#define PG14  PIN_DEF(GPIOG, 14)
#define PG15  PIN_DEF(GPIOG, 15)
#endif

#ifdef GPIOH
#define PH0   PIN_DEF(GPIOH, 0)
#define PH1   PIN_DEF(GPIOH, 1)
#define PH2   PIN_DEF(GPIOH, 2)
#define PH3   PIN_DEF(GPIOH, 3)
#define PH4   PIN_DEF(GPIOH, 4)
#define PH5   PIN_DEF(GPIOH, 5)
#define PH6   PIN_DEF(GPIOH, 6)
#define PH7   PIN_DEF(GPIOH, 7)
#define PH8   PIN_DEF(GPIOH, 8)
#define PH9   PIN_DEF(GPIOH, 9)
#define PH10  PIN_DEF(GPIOH, 10)
#define PH11  PIN_DEF(GPIOH, 11)
#define PH12  PIN_DEF(GPIOH, 12)
#define PH13  PIN_DEF(GPIOH, 13)
#define PH14  PIN_DEF(GPIOH, 14)
#define PH15  PIN_DEF(GPIOH, 15)
#endif

#ifdef GPIOI
#define PI0   PIN_DEF(GPIOI, 0)
#define PI1   PIN_DEF(GPIOI, 1)
#define PI2   PIN_DEF(GPIOI, 2)
#define PI3   PIN_DEF(GPIOI, 3)
#define PI4   PIN_DEF(GPIOI, 4)
#define PI5   PIN_DEF(GPIOI, 5)
#define PI6   PIN_DEF(GPIOI, 6)
#define PI7   PIN_DEF(GPIOI, 7)
#define PI8   PIN_DEF(GPIOI, 8)
#define PI9   PIN_DEF(GPIOI, 9)
#define PI10  PIN_DEF(GPIOI, 10)
#define PI11  PIN_DEF(GPIOI, 11)
#define PI12  PIN_DEF(GPIOI, 12)
#define PI13  PIN_DEF(GPIOI, 13)
#define PI14  PIN_DEF(GPIOI, 14)
#define PI15  PIN_DEF(GPIOI, 15)
#endif

#ifdef GPIOJ
#define PJ0   PIN_DEF(GPIOJ, 0)
#define PJ1   PIN_DEF(GPIOJ, 1)
#define PJ2   PIN_DEF(GPIOJ, 2)
#define PJ3   PIN_DEF(GPIOJ, 3)
#define PJ4   PIN_DEF(GPIOJ, 4)
#define PJ5   PIN_DEF(GPIOJ, 5)
#define PJ6   PIN_DEF(GPIOJ, 6)
#define PJ7   PIN_DEF(GPIOJ, 7)
#define PJ8   PIN_DEF(GPIOJ, 8)
#define PJ9   PIN_DEF(GPIOJ, 9)
#define PJ10  PIN_DEF(GPIOJ, 10)
#define PJ11  PIN_DEF(GPIOJ, 11)
#define PJ12  PIN_DEF(GPIOJ, 12)
#define PJ13  PIN_DEF(GPIOJ, 13)
#define PJ14  PIN_DEF(GPIOJ, 14)
#define PJ15  PIN_DEF(GPIOJ, 15)
#endif

#ifdef GPIOK
#define PK0   PIN_DEF(GPIOK, 0)
#define PK1   PIN_DEF(GPIOK, 1)
#define PK2   PIN_DEF(GPIOK, 2)
#define PK3   PIN_DEF(GPIOK, 3)
#define PK4   PIN_DEF(GPIOK, 4)
#define PK5   PIN_DEF(GPIOK, 5)
#define PK6   PIN_DEF(GPIOK, 6)
#define PK7   PIN_DEF(GPIOK, 7)
#define PK8   PIN_DEF(GPIOK, 8)
#define PK9   PIN_DEF(GPIOK, 9)
#define PK10  PIN_DEF(GPIOK, 10)
#define PK11  PIN_DEF(GPIOK, 11)
#define PK12  PIN_DEF(GPIOK, 12)
#define PK13  PIN_DEF(GPIOK, 13)
#define PK14  PIN_DEF(GPIOK, 14)
#define PK15  PIN_DEF(GPIOK, 15)
#endif

/* Rare but present on some H7 variants */
#ifdef GPIOZ
#define PZ0   PIN_DEF(GPIOZ, 0)
#define PZ1   PIN_DEF(GPIOZ, 1)
#define PZ2   PIN_DEF(GPIOZ, 2)
#define PZ3   PIN_DEF(GPIOZ, 3)
#define PZ4   PIN_DEF(GPIOZ, 4)
#define PZ5   PIN_DEF(GPIOZ, 5)
#define PZ6   PIN_DEF(GPIOZ, 6)
#define PZ7   PIN_DEF(GPIOZ, 7)
#define PZ8   PIN_DEF(GPIOZ, 8)
#define PZ9   PIN_DEF(GPIOZ, 9)
#define PZ10  PIN_DEF(GPIOZ, 10)
#define PZ11  PIN_DEF(GPIOZ, 11)
#define PZ12  PIN_DEF(GPIOZ, 12)
#define PZ13  PIN_DEF(GPIOZ, 13)
#define PZ14  PIN_DEF(GPIOZ, 14)
#define PZ15  PIN_DEF(GPIOZ, 15)
#endif

/* -------------------------------------------------
 * Digital I/O API
 * ------------------------------------------------- */

static inline void digital_write(dpin_t p, DigitalLevel state)
{
    HAL_GPIO_WritePin(p.port, p.pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline DigitalLevel digital_read(dpin_t p)
{
    return (HAL_GPIO_ReadPin(p.port, p.pin) == GPIO_PIN_SET) ? HIGH : LOW;
}

static inline void digital_toggle(dpin_t p)
{
    HAL_GPIO_TogglePin(p.port, p.pin);
}

#ifdef __cplusplus
}
#endif

#endif /* DIGITAL_IO_H */
