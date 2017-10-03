#ifndef BI3PLUSLCD_H
#define BI3PLUSLCD_H

#include "Marlin.h"
#include "utils.hpp"

namespace marlin::lcd
{
	void initialize();
	void update();
	void show_page(uint8 pageNumber);
	void update_graph();
}

#endif
