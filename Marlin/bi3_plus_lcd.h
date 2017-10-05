#ifndef BI3PLUSLCD_H
#define BI3PLUSLCD_H

#include "Marlin.h"
#include <tuna>

namespace marlin::lcd
{
	void initialize();
	void update();
	void show_page(uint8 pageNumber);
	void update_graph();
	constexpr inline bool has_status() { return false; }
	constexpr inline void set_status(const char* const, const bool = false) { }
	constexpr inline void set_status_PGM(const char* const, const int8_t = 0) { }
	constexpr inline void set_alert_status_PGM(const char*) { }
	constexpr inline void statusf(const uint8_t, const char * const, ...) { }
	constexpr inline void update_buttons() {}
	constexpr inline void reset_alert_level() {}
	constexpr inline void refresh() {}
}

#define LCD_MESSAGEPGM(x)      lcd::set_status_PGM(PSTR(x))
#define LCD_ALERTMESSAGEPGM(x) lcd::set_alert_status_PGM(PSTR(x))

#endif
