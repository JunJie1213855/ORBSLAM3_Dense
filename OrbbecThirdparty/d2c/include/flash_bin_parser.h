#pragma once

#include "depth_to_color.h"


class Flash_bin_parser {

public:

	Flash_bin_parser();

	int getDataSize();

	int parse_buffer_to_params(char* buffer, SW_D2C* soft_d2c);

	int parse_bin_to_params(char* bin_file, SW_D2C* soft_d2c);


};
