#include "datetime.h"

static struct datetime current_datetime;

void datetime_set(struct datetime *dt)
{
	current_datetime = *dt;
}

struct datetime datetime_now(void)
{
	return current_datetime;
}

static bool datetime_day_ge(struct datetime *dt, struct datetime *point)
{
	if (dt->hours > point->hours)
		return true;
	else if (dt->hours == point->hours)
	{
		if (dt->minutes > point->minutes)
			return true;
		else if (dt->minutes == point->minutes)
		{
			if (dt->seconds >= point->seconds)
				return true;
		}
	}

	return false;
}

static bool datetime_month_ge(struct datetime *dt, struct datetime *point)
{
	if (dt->day > point->day)
		return true;
	else if (dt->day == point->day)
		return datetime_day_ge(dt, point);

	return false;
}

static bool datetime_year_ge(struct datetime *dt, struct datetime *point)
{
	if (dt->month > point->month)
		return true;
	else if (dt->month == point->month)
		return datetime_month_ge(dt, point);

	return false;
}

static bool datetime_strict_ge(struct datetime *dt, struct datetime *point)
{
	if (dt->year > point->year)
		return true;
	else if (dt->year == point->year)
		return datetime_year_ge(dt, point);

	return false;
}

static bool datetime_day_le(struct datetime *dt, struct datetime *point)
{
	if (dt->hours < point->hours)
		return true;
	else if (dt->hours == point->hours)
	{
		if (dt->minutes < point->minutes)
			return true;
		else if (dt->minutes == point->minutes)
		{
			if (dt->seconds <= point->seconds)
				return true;
		}
	}

	return false;
}

static bool datetime_month_le(struct datetime *dt, struct datetime *point)
{
	if (dt->day < point->day)
		return true;
	else if (dt->day == point->day)
		return datetime_day_le(dt, point);

	return false;
}

static bool datetime_year_le(struct datetime *dt, struct datetime *point)
{
	if (dt->month < point->month)
		return true;
	else if (dt->month == point->month)
		return datetime_month_le(dt, point);

	return false;
}

static bool datetime_strict_le(struct datetime *dt, struct datetime *point)
{
	if (dt->year < point->year)
		return true;
	else if (dt->year == point->year)
		return datetime_year_le(dt, point);

	return false;
}

bool datetime_in(	struct datetime *dt, 
					struct datetime *point1, struct datetime *point2, 
					enum time_constrains constrains)
{
	switch (constrains)
	{
	case ALL_TIME:
		return true;
	case STRICT_EQUALITY:
		return (datetime_strict_ge(dt, point1) && datetime_strict_le(dt, point2));
	case EVERY_DAY:
		return (datetime_day_ge(dt, point1) && datetime_day_le(dt, point2));
	case EVERY_MONTH:
		return (datetime_month_ge(dt, point1) && datetime_month_le(dt, point2));
	case EVERY_YEAR:
		return (datetime_year_ge(dt, point1) && datetime_year_le(dt, point2));
	}

	return false;
}
