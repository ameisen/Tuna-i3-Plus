using uint8 = unsigned char;
static_assert(sizeof(uint8) == 1, "ohno");
using uint16 = unsigned int;
static_assert(sizeof(uint16) == 2, "ohno");
using uint32 = unsigned long;
static_assert(sizeof(uint32) == 4, "ohno");

struct out_t final
{
	uint32 boolean : 1;
	uint32 value1 : 10;
	uint32 value2 : 10;
};

inline uint16 min (uint16 val, uint16 other_val)
{
	return val <= other_val ? val : other_val;
}

int main ()
{	
	volatile uint16 in;
	volatile out_t out;
	
	constexpr const uint16 comp_value = 1024;
	
	out_t outv;
	
	outv.value1 = min(in, comp_value);
	outv.value2 = min(in, comp_value);

	outv.boolean = (outv.value1 < 512);
	
	out = outv;

	return 0;
}