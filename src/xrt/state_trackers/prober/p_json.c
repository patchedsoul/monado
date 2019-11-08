// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Code to manage the settings file.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup st_prober
 */

#include "util/u_json.h"
#include "util/u_debug.h"
#include "p_prober.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>



static FILE *
get_file(const char *mode)
{
	char path_string[PATH_MAX];
	char file_string[PATH_MAX + 15];

	const char *xgd_home = getenv("XDG_CONFIG_HOME");
	const char *home = getenv("HOME");
	if (xgd_home != NULL) {
		snprintf(path_string, PATH_MAX, "%s/monado", xgd_home);
	} else if (home != NULL) {
		snprintf(path_string, PATH_MAX, "%s/.config/monado", home);
	} else {
		return NULL;
	}

	snprintf(file_string, PATH_MAX + 15, "%s/config_v0.json", path_string);
	FILE *file = fopen(file_string, mode);
	if (file != NULL) {
		return file;
	}

	// Try creating the path.
	mkdir(path_string, S_IRWXU);

	file = fopen(file_string, mode);
	if (file == NULL) {
		fprintf(stderr, "Could not create config file %s\n",
		        file_string);
	}

	return file;
}

char *
read_content(FILE *file)
{
	// Go to the end of the file.
	fseek(file, 0L, SEEK_END);
	size_t file_size = ftell(file);

	// Return back to the start of the file.
	fseek(file, 0L, SEEK_SET);

	char *buffer = (char *)calloc(file_size, sizeof(char));
	if (buffer == NULL) {
		return NULL;
	}

	// Do the actual reading.
	size_t ret = fread(buffer, sizeof(char), file_size, file);
	if (ret != file_size) {
		free(buffer);
		return NULL;
	}

	return buffer;
}

cJSON *
p_json_open_or_create_main_file(void)
{
	FILE *file = get_file("r");
	if (file == NULL) {
		fprintf(stderr, "Could not open the file!\n");
		return NULL;
	}

	char *str = read_content(file);
	fclose(file);
	if (str == NULL) {
		fprintf(stderr, "Could not read the contents of the file!\n");
		return NULL;
	}

	cJSON *ret = cJSON_Parse(str);

	const char *content = cJSON_Print(ret);
	fprintf(stderr, "%s\n", content);

	free(str);
	return ret;
}
