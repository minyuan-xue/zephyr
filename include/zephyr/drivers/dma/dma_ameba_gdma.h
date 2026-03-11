/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_AMEBA_GDMA_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_AMEBA_GDMA_H_

/**
 * @file
 * @brief Ameba GDMA helper macros for devicetree-based configuration
 *
 * This file provides helper macros to get DMA controllers and
 * configuration values from devicetree, and to translate packed
 * configuration fields into DMA API parameters.
 */

/**
 * @brief Get DMA controller device for an instance and named DMA spec.
 *
 * @param n    Devicetree instance index.
 * @param name Name of the DMA spec (e.g. "tx", "rx").
 */
#define AMEBA_DT_INST_DMA_CTLR(n, name)                                                            \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),                                                \
		    (DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, name))), (NULL))

/**
 * @brief Get a cell value from a named DMA spec for an instance.
 *
 * @param n    Devicetree instance index.
 * @param name Name of the DMA spec (e.g. "tx", "rx").
 * @param cell Name of the cell in the spec (e.g. "slot", "config").
 */
#define AMEBA_DT_INST_DMA_CELL(n, name, cell)                                                      \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas), (DT_INST_DMAS_CELL_BY_NAME(n, name, cell)),    \
		    (0xff))

/* macros for getting channel-configuration */

/**
 * @brief Direction defined on bits 0-1.
 *
 * @param value Packed configuration value:
 *              0 -> MEM_TO_MEM,
 *              1 -> MEM_TO_PERIPH,
 *              2 -> PERIPH_TO_MEM,
 *              3 -> PERIPH_TO_PERIPH.
 */
#define AMEBA_DMA_CH_DIR_GET(value) ((value >> 0) & 0x3)

/**
 * @brief Source address increment mode defined on bits 2-3.
 *
 * @param value Packed configuration value:
 *              0 -> increment,
 *              1 -> decrement (not supported),
 *              2 -> no change.
 */
#define AMEBA_DMA_SRC_ADDR_ADJ_GET(value) ((value >> 2) & 0x3)

/**
 * @brief Destination address increment mode defined on bits 4-5.
 *
 * @param value Packed configuration value:
 *              0 -> increment,
 *              1 -> decrement (not supported),
 *              2 -> no change.
 */
#define AMEBA_DMA_DST_ADDR_ADJ_GET(value) ((value >> 4) & 0x3)

/**
 * @brief Source data size defined on bits 6-8.
 *
 * @param value Packed configuration value (width can be 1/2/4 bytes).
 */
#define AMEBA_DMA_SRC_DATA_SIZE_GET(value) ((value >> 6) & 0x7)

/**
 * @brief Destination data size defined on bits 9-11.
 *
 * @param value Packed configuration value (width can be 1/2/4 bytes).
 */
#define AMEBA_DMA_DST_DATA_SIZE_GET(value) ((value >> 9) & 0x7)

/**
 * @brief Source burst size (msize) defined on bits 12-16.
 *
 * @param value Packed configuration value (burst size can be 1, 4, 8, 16).
 */
#define AMEBA_DMA_SRC_BST_SIZE_GET(value) ((value >> 12) & 0x1F)

/**
 * @brief Destination burst size (msize) defined on bits 17-21.
 *
 * @param value Packed configuration value (burst size can be 1, 4, 8, 16).
 */
#define AMEBA_DMA_DST_BST_SIZE_GET(value) ((value >> 17) & 0x1F)

/**
 * @brief Channel priority defined on bits 22-24 as 0-7.
 *
 * @param value Packed configuration value (priority 0-7, 0 is highest).
 */
#define AMEBA_DMA_CH_PRIORITY_GET(value) ((value >> 22) & 0x7)

/**
 * @brief Build a DMA configuration initializer from devicetree.
 *
 * @param index        Devicetree instance index.
 * @param dir          Name of the DMA spec (e.g. "tx", "rx").
 * @param block_number Number of DMA blocks.
 * @param cb           DMA callback function.
 */
#define AMEBA_DMA_CONFIG(index, dir, block_number, cb)                                             \
	{                                                                                          \
		.dma_slot = DT_INST_DMAS_CELL_BY_NAME(index, dir, slot),                           \
		.channel_direction =                                                               \
			AMEBA_DMA_CH_DIR_GET(AMEBA_DT_INST_DMA_CELL(index, dir, config)),          \
		.channel_priority =                                                                \
			AMEBA_DMA_CH_PRIORITY_GET(AMEBA_DT_INST_DMA_CELL(index, dir, config)),     \
		.source_data_size =                                                                \
			AMEBA_DMA_SRC_DATA_SIZE_GET(AMEBA_DT_INST_DMA_CELL(index, dir, config)),   \
		.dest_data_size =                                                                  \
			AMEBA_DMA_DST_DATA_SIZE_GET(AMEBA_DT_INST_DMA_CELL(index, dir, config)),   \
		.source_burst_length =                                                             \
			AMEBA_DMA_SRC_BST_SIZE_GET(AMEBA_DT_INST_DMA_CELL(index, dir, config)),    \
		.dest_burst_length =                                                               \
			AMEBA_DMA_DST_BST_SIZE_GET(AMEBA_DT_INST_DMA_CELL(index, dir, config)),    \
		.block_count = block_number,                                                       \
		.dma_callback = cb,                                                                \
	}

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_AMEBA_GDMA_H_ */
