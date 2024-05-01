/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/audio/cap.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/kernel.h>

#define SINK_CONTEXT        BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED
#define SOURCE_CONTEXT      BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED

/** Struct to contain information for a specific peer (CAP) device */
struct peer_config {
	/** Stream for the source endpoint */
	struct bt_cap_stream source_stream;
	/** Stream for the sink endpoint */
	struct bt_cap_stream sink_stream;
	/** Semaphore to help wait for a release operation if the source stream is not idle */
	struct k_sem source_stream_sem;
	/** Semaphore to help wait for a release operation if the sink stream is not idle */
	struct k_sem sink_stream_sem;
	/** ACL connection object for the peer device */
	struct bt_conn *conn;
};

/**
 * @brief Initialize the unicast part of the CAP Acceptor
 *
 * @retval 0 if success
 * @retval -ENOEXEC if callbacks failed to be registered
 */
int init_cap_acceptor_unicast(struct peer_config *peer);

/**
 * @brief Request to allocate a CAP stream
 *
 * @retval Pointer to the allocated CAP stream
 * @retval NULL if no more CAP streams for the @p dir could be allocated
 */
struct bt_cap_stream *stream_alloc(enum bt_audio_dir dir);

void stream_released(const struct bt_cap_stream *cap_stream);
