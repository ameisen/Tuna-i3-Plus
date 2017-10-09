#pragma once

#include "tunalib/utils.hpp"

namespace tuna
{
	template <typename T, uint8 fraction_bits>
	class fixed final
	{
		constexpr static const uint8 total_bits = sizeof(T) * 8;
		constexpr static const uint8 integer_bits = total_bits - fraction_bits;
		static_assert(fraction_bits <= total_bits, "Fraction bits cannot be larger than the datatype size");
		static_assert(fraction_bits >= 1, "Must have at least one fraction bit... otherwise this is just an integer");
	
		union data_pun
		{
			T value;
			struct
			{
				T fraction : fraction_bits;
				T integer : integer_bits;
			};

			constexpr data_pun(T _value) : value(_value) {}
		} m_Data;
		static_assert(sizeof(m_Data) == sizeof(T), "Data structure does not match datatype size. Not good.");

		using bigint_t = typename type_trait<T>::larger_type;

		// internal constructor used to construct from raw.
		constexpr fixed(T val, bool) : m_Data(val) {}

	public:
		using type = T;

		static constexpr uint8 frac_bits = fraction_bits;
		static constexpr uint8 int_bits = integer_bits;

		constexpr static fixed _from(T val)
		{
			return { val, true };
		}

		// ::raw returns a structure as otherwise it's really painful to constexpr this.
		struct raw_t
		{
			uint16_t value;
		};
		constexpr raw_t raw() const
		{

			return { m_Data.value << fraction_bits };
		}

		constexpr operator raw_t () const
		{
			return raw();
		}

		constexpr fixed() = default;
		constexpr fixed(const fixed &val) : m_Data(val.m_Data) {}
		// Move constructor/operators seem unnecessary since it stores POD.
		template <typename A>
		constexpr fixed(A val) : m_Data(
			(sizeof(T) > sizeof(A)) ?
			T(((typename type_trait<A>::larger_type)(val)) << fraction_bits) :
			m_Data.value = val << fraction_bits
		)
		{
			static_assert(type_trait<A>::is_integral, "constructor argument must be an integer or a float.");
		}
		constexpr fixed(float val) : m_Data(
			T((val * float(T(1) << fraction_bits)) + 0.5f)
		)
		{}

		constexpr fixed & operator = (const fixed &val)
		{
			m_Data = val.m_Data;
			return *this;
		}

		template <typename A>
		constexpr fixed & operator = (A val)
		{
			static_assert(type_trait<A>::is_integral, "assignment argument must be an integer or a float.");
			if constexpr (sizeof(T) > sizeof(A))
			{
				using conv_t = typename type_trait<A>::larger_type;
				m_Data.value = conv_t(val) << fraction_bits;
				return *this;
			}
			else
			{
				m_Data.value = val << fraction_bits;
				return *this;
			}
		}

		constexpr fixed & operator = (float val)
		{
			constexpr const T multiplicand = T(1) << fraction_bits;
			constexpr const float multiplicandf = float(multiplicand);
			val *= multiplicandf;
			m_Data.value = T(val + 0.5f);

			return *this;
		}

		constexpr operator float() const
		{
			union punner
			{
				bigint_t value;
				struct
				{
					bigint_t fraction : fraction_bits;
					bigint_t integer : integer_bits + sizeof(T);
				};

				constexpr punner(bigint_t integ, bigint_t frac) : fraction(frac), integer(integ) {}
			} _values{ m_Data.integer, m_Data.fraction };

			return float(_values.integer) + (float(_values.fraction) / float(T{ 1 } << fraction_bits));
		}

		constexpr operator T() const
		{
			return m_Data.integer + (m_Data.fraction >> (fraction_bits - 1));
		}

		constexpr bool operator == (fixed val) const
		{
			return m_Data.value == val.m_Data.value;
		}

		constexpr bool operator != (fixed val) const
		{
			return m_Data.value != val.m_Data.value;
		}

		constexpr bool operator > (fixed val) const
		{
			return m_Data.value > val.m_Data.value;
		}

		constexpr bool operator >= (fixed val) const
		{
			return m_Data.value >= val.m_Data.value;
		}

		constexpr bool operator < (fixed val) const
		{
			return m_Data.value < val.m_Data.value;
		}

		constexpr bool operator <= (fixed val) const
		{
			return m_Data.value <= val.m_Data.value;
		}

		constexpr bool operator == (T val) const
		{
			return m_Data.integer == val;
		}

		constexpr bool operator != (T val) const
		{
			return m_Data.integer != val;
		}

		constexpr bool operator > (T val) const
		{
			return m_Data.integer > val;
		}

		constexpr bool operator >= (T val) const
		{
			return m_Data.integer >= val;
		}

		constexpr bool operator < (T val) const
		{
			return m_Data.integer < val;
		}

		constexpr bool operator <= (T val) const
		{
			return m_Data.integer <= val;
		}

		constexpr fixed & operator *= (fixed val)
		{
			const T y = val.m_Data.value;
			const T x = m_Data.value;
			m_Data.value = (bigint_t(x) * y) >> fraction_bits;
			return *this;
		}

		constexpr fixed & operator /= (fixed val)
		{
			const T y = val.m_Data.value;
			const T x = m_Data.value;

			m_Data.value = (bigint_t(x) << fraction_bits) / y;
			return *this;
		}

		constexpr fixed & operator %= (fixed val)
		{
			const T y = val.m_Data.value;
			const T x = m_Data.value;

			m_Data.value = (bigint_t(x) << fraction_bits) % y;
			return *this;
		}

		constexpr fixed & operator += (fixed val)
		{
			m_Data.value += val.m_Data.value;
			return *this;
		}

		constexpr fixed & operator -= (fixed val)
		{
			m_Data.value -= val.m_Data.value;
			return *this;
		}

		constexpr fixed operator * (fixed val) const
		{
			const T y = val.m_Data.value;
			const T x = m_Data.value;
			return _from((bigint_t(x) * y) >> fraction_bits);
		}

		constexpr fixed operator / (fixed val) const
		{
			const T y = val.m_Data.value;
			const T x = m_Data.value;
			return _from((bigint_t(x) << fraction_bits) / y);
		}

		constexpr fixed operator % (fixed val) const
		{
			const T y = val.m_Data.value;
			const T x = m_Data.value;
			return _from((bigint_t(x) << fraction_bits) % y);
		}

		constexpr fixed operator + (fixed val) const
		{
			return _from(m_Data.value + val.m_Data.value);
		}

		constexpr fixed operator - (fixed val) const
		{
			return _from(m_Data.value - val.m_Data.value);
		}

		template <typename A>
		constexpr fixed & operator *= (A val)
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			m_Data.value *= val;
			return *this;
		}

		template <typename A>
		constexpr fixed & operator /= (A val)
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			m_Data.value /= val;
			return *this;
		}

		template <typename A>
		constexpr fixed & operator %= (A val)
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			m_Data.value %= val;
			return *this;
		}

		template <typename A>
		constexpr fixed & operator += (A val)
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			m_Data.integer += val;
			return *this;
		}

		template <typename A>
		constexpr fixed & operator -= (A val)
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			m_Data.integer -= val;
			return *this;
		}

		template <typename A>
		constexpr fixed operator * (A val) const
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			return _from(m_Data.value * val);
		}

		template <typename A>
		constexpr fixed operator / (A val) const
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			return _from(m_Data.value / val);
		}

		template <typename A>
		constexpr fixed operator % (A val) const
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			return _from(m_Data.value % val);
		}

		template <typename A>
		constexpr fixed operator + (A val) const
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			auto value = m_Data;
			value.integer += val;
			return _from(value.value);
		}

		template <typename A>
		constexpr fixed operator - (A val) const
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			auto value = m_Data;
			value.integer -= val;
			return _from(value.value);
		}

		fixed rounded() const
		{
			auto value = m_Data;
			constexpr const uint8 half = 1_u8 << (fraction_bits - 1);
			value.value += half;
			value.fraction = 0;
			return _from(value.value);
		}
	};
}
