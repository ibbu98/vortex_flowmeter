Designed and implemented low-level Assembly and C routines to evaluate memory utilization and execution efficiency on an ARM Cortex-M4 microcontroller.
Built a bare-metal vortex flowmeter program on STM32F401RE using pre-given ADC data to detect vortex frequency at 10 kHz sampling.
Converted ADC readings into temperature and used that to compute water density and viscosity.
Calculated Reynolds number and Strouhal number from fluid properties to estimate flow velocity from vortex frequency.
Converted velocity into volumetric flow rate (GPM) using pipe diameter and cross-sectional area formulas.
Displayed results on an SPI-based LCD, transmitted data over UART, and generated PWM outputs to mimic industrial flow signals.
