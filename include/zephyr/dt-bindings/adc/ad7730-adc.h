/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Zoltan Gyarmati
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_ADC_AD7730_ADC_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_ADC_AD7730_ADC_H_

/*
 * AD7730 channel-pair selection (CH[1:0] in the Mode Register, datasheet
 * Table XIII). The AD7730 has no freely configurable analog input mux —
 * each channel_id selects one of these four dedicated pairings directly,
 * so these values are used as a channel node's `reg` property.
 */
#define AD7730_CH_1         0 /* AIN1(+)/AIN1(-), Calibration Register Pair 0 */
#define AD7730_CH_2         1 /* AIN2(+)/AIN2(-), Calibration Register Pair 1 */
#define AD7730_CH_1_SHORTED 2 /* AIN1(-) shorted to AIN1(-); noise self-test, Pair 0 */
#define AD7730_CH_1_AIN2    3 /* AIN1(+)/AIN2(-), Calibration Register Pair 2 */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_ADC_AD7730_ADC_H_ */
