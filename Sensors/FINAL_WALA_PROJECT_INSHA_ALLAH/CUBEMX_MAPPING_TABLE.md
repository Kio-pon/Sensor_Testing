# Master CubeMX Mapping Table

Use this exact checklist when assigning the 21 new pins in STM32CubeMX. Go down the list row by row.

| Physical Pin | Set Mode To... | User Label (Right-click -> Enter User Label) | Specific Setting in Left Panel |
| :--- | :--- | :--- | :--- |
| **PA0** | `ADC1_IN1` | `FRONT_QTR_1` | **ADC1:** Set Sampling Time to 61.5 Cycles |
| **PA1** | `ADC1_IN2` | `FRONT_QTR_2` | **ADC1:** Set Sampling Time to 61.5 Cycles |
| **PA2** | `ADC1_IN3` | `FRONT_QTR_3` | **ADC1:** Set Sampling Time to 61.5 Cycles |
| **PA3** | `ADC1_IN4` | `FRONT_QTR_4` | **ADC1:** Set Sampling Time to 61.5 Cycles |
| **PA4** | `ADC2_IN1` | `FRONT_QTR_5` | **ADC2:** Set Sampling Time to 61.5 Cycles |
| **PC0** | `ADC1_IN6` *(or ADC12_IN6)* | `FRONT_QTR_6` | **ADC1:** Set Sampling Time to 61.5 Cycles |
| **PC1** | `ADC1_IN7` *(or ADC12_IN7)* | `FRONT_QTR_7` | **ADC1:** Set Sampling Time to 61.5 Cycles |
| **PC2** | `ADC1_IN8` *(or ADC12_IN8)* | `FRONT_QTR_8` | **ADC1:** Set Sampling Time to 61.5 Cycles |
| **PC3** | `ADC2_IN9` *(or ADC12_IN9)* | `SHARP_IR` | **ADC2:** Set Sampling Time to 61.5 Cycles |
| | | | |
| **PD13** | `GPIO_Input` | `LEFT_BFD_1` | **GPIO:** Set to Pull-Up |
| **PD14** | `GPIO_Input` | `LEFT_BFD_2` | **GPIO:** Set to Pull-Up |
| **PD11** | `GPIO_Input` | `LEFT_BFD_3` | **GPIO:** Set to Pull-Up |
| **PC10** | `GPIO_Input` | `LEFT_BFD_4` | **GPIO:** Set to Pull-Up |
| **PC11** | `GPIO_Input` | `LEFT_BFD_5` | **GPIO:** Set to Pull-Up |
| | | | |
| **PD10** | `GPIO_Input` | `RIGHT_BFD_1` | **GPIO:** Set to Pull-Up |
| **PF2**  | `GPIO_Input` | `RIGHT_BFD_2` | **GPIO:** Set to Pull-Up |
| **PE6** | `GPIO_Input` | `RIGHT_BFD_3` | **GPIO:** Set to Pull-Up |
| **PE7** | `GPIO_Input` | `RIGHT_BFD_4` | **GPIO:** Set to Pull-Up |
| **PF4** | `GPIO_Input` | `RIGHT_BFD_5` | **GPIO:** Set to Pull-Up |
| | | | |
| **PB10** | `USART3_TX` | `ARDUINO_TX` | **USART3:** Asynchronous, 115200 Baud |
| **PB11** | `USART3_RX` | `ARDUINO_RX` | **USART3:** Asynchronous, 115200 Baud |

---

### Final CubeMX Checklist Before Generating:
- [ ] **ADC1 and ADC2:** Ensure `Continuous Conversion Mode` is **Disabled**.
- [ ] **USART3:** Go to NVIC Settings and enable the **USART3 global interrupt**.
- [ ] **TIM6:** Verify it is still active with PSC set for 1MHz and ARR = 49. **TIM6 global interrupt MUST be enabled**.
- [ ] **TIM2:** Verify CH1-4 are still set to PWM Generation with ARR = 4799.
