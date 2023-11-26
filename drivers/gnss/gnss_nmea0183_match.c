/*
 * Copyright (c) 2023 Trackunit Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/kernel.h>
#include <zephyr/modem/chat.h>

#include <string.h>

#include "gnss_nmea0183.h"
#include "gnss_nmea0183_match.h"

static bool gnss_nmea0183_match_timed_out(struct gnss_nmea0183_match_data *data)
{
	int64_t delta;

	delta = k_uptime_delta(&data->timestamp);
	data->timestamp = k_uptime_get();
	return ((uint16_t)delta) > data->timeout_ms;
}

#if CONFIG_GNSS_SATELLITES
static void gnss_nmea0183_match_reset_gsv(struct gnss_nmea0183_match_data *data)
{
	data->satellites_length = 0;
	data->gsv_message_number = 1;
}
#endif

static void gnss_nmea0183_match_reset(struct gnss_nmea0183_match_data *data)
{
	data->gga_received = false;
	data->rmc_received = false;
}

void gnss_nmea0183_match_gga_callback(struct modem_chat *chat, char **argv, uint16_t argc,
				      void *user_data)
{
	struct gnss_nmea0183_match_data *data = user_data;

	if (gnss_nmea0183_match_timed_out(data)) {
		gnss_nmea0183_match_reset(data);
	}

	if (gnss_nmea0183_parse_gga((const char **)argv, argc, &data->data) < 0) {
		return;
	}

	data->gga_received = true;

	if (data->gga_received && data->rmc_received) {
		gnss_publish_data(data->gnss, &data->data);
	}
}

void gnss_nmea0183_match_rmc_callback(struct modem_chat *chat, char **argv, uint16_t argc,
				      void *user_data)
{
	struct gnss_nmea0183_match_data *data = user_data;

	if (gnss_nmea0183_match_timed_out(data)) {
		gnss_nmea0183_match_reset(data);
	}

	if (gnss_nmea0183_parse_rmc((const char **)argv, argc, &data->data) < 0) {
		return;
	}

	data->rmc_received = true;

	if (data->gga_received && data->rmc_received) {
		gnss_publish_data(data->gnss, &data->data);
	}
}

#if CONFIG_GNSS_SATELLITES
void gnss_nmea0183_match_gsv_callback(struct modem_chat *chat, char **argv, uint16_t argc,
				      void *user_data)
{
	struct gnss_nmea0183_match_data *data = user_data;
	struct gnss_nmea0183_gsv_header header;
	int ret;

	if (gnss_nmea0183_match_timed_out(data)) {
		gnss_nmea0183_match_reset(data);
	}

	if (gnss_nmea0183_parse_gsv_header((const char **)argv, argc, &header) < 0) {
		return;
	}

	if (header.number_of_svs == 0) {
		return;
	}

	if (header.message_number != data->gsv_message_number) {
		gnss_nmea0183_match_reset_gsv(data);
		return;
	}

	data->gsv_message_number++;

	ret = gnss_nmea0183_parse_gsv_svs((const char **)argv, argc,
					  &data->satellites[data->satellites_length],
					  data->satellites_size - data->satellites_length);
	if (ret < 0) {
		gnss_nmea0183_match_reset_gsv(data);
		return;
	}

	data->satellites_length += (uint16_t)ret;

	if (data->satellites_length == header.number_of_svs) {
		gnss_publish_satellites(data->gnss, data->satellites, data->satellites_length);
		gnss_nmea0183_match_reset_gsv(data);
	}
}
#endif

int gnss_nmea0183_match_init(struct gnss_nmea0183_match_data *data,
			     const struct gnss_nmea0183_match_config *config)
{
	__ASSERT(data != NULL, "data argument must be provided");
	__ASSERT(config != NULL, "data argument must be provided");

	memset(data, 0, sizeof(struct gnss_nmea0183_match_data));
	data->gnss = config->gnss;
#if CONFIG_GNSS_SATELLITES
	data->satellites = config->satellites;
	data->satellites_size = config->satellites_size;
#endif
	data->timeout_ms = config->timeout_ms;
	return 0;
}
