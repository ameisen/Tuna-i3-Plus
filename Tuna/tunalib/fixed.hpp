#pragma once

#include "tunalib/utils.hpp"

namespace Tuna
{
	template <typename T, uint8 fraction_bits>
	class fixed final
	{
	public:
		using type = T;

		static constexpr const uint8 total_bits = sizeof(T) * 8;
		static constexpr const uint8 fractional_bits = fraction_bits;
		static constexpr const uint8 integer_bits = total_bits - fractional_bits;
	private:
		static_assert(fractional_bits <= total_bits, "Fraction bits cannot be larger than the datatype size");
		static_assert(fractional_bits >= 1, "Must have at least one fraction bit... otherwise this is just an integer");
	
		using integer_t = uintsz<(1_u64 << integer_bits) - 1>;
		using fractional_t = uintsz<(1_u64 << fractional_bits) - 1>;

		// I'd much rather use a union here - however, to maintain constexpr compatibility, a union cannot be used
		// as in constexpr, inactive/active membership is strictly enforced - that is - no type punning.
		T m_Value;

		using bigint_t = typename type_trait<T>::larger_type;

		// internal constructor used to construct from raw.
		constexpr fixed(arg_type<T> val, bool) : m_Value(val) {}

		template <typename U = integer_t>
		constexpr inline U get_integer() const __restrict
		{
			constexpr const U integer_mask = (U{ 1 } << integer_bits) - 1;
			if constexpr (integer_mask >= type_trait<U>::max)
			{
				// truncation suffices as the return type is more restrictive than the mask.
				return m_Value >> fractional_bits;
			}
			else
			{
				return (m_Value >> fractional_bits) & integer_mask;
			}
		}

		template <typename U = fractional_t>
		constexpr inline U get_fraction() const __restrict
		{
			constexpr const U fractional_mask = (U{ 1 } << fractional_bits) - 1;
			if constexpr (fractional_mask >= type_trait<U>::max)
			{
				// truncation suffices as the return type is more restrictive than the mask.
				return m_Value;
			}
			else
			{
				return (m_Value & fractional_mask);
			}
		}

		template <typename U = integer_t>
		constexpr inline void set_integer(arg_type<U> value) __restrict
		{
			constexpr const T integer_mask = (T{ 1 } << integer_bits) - 1;
			m_Value ^= ((m_Value ^ (T(value) << fractional_bits)) & integer_mask);
		}

		template <typename U = fractional_t>
		constexpr inline void set_fraction(arg_type<U> value) __restrict
		{
			if constexpr (type_trait<U>::bits >= fractional_bits)
			{
				constexpr const U fractional_mask = (U{ 1 } << fractional_bits) - 1;
				m_Value ^= ((m_Value ^ value) & fractional_mask);
			}
			else
			{
				constexpr const T fractional_mask = (T{ 1 } << fractional_bits) - 1;
				m_Value ^= ((m_Value ^ T(value)) & fractional_mask);
			}
		}

	public:
		constexpr static fixed from(arg_type<T> val)
		{
			return { val, true };
		}

		constexpr const type & __restrict raw() const __restrict
		{
			return m_Value;
		}

		constexpr fixed() = default;
		constexpr fixed(arg_type<fixed> val) : m_Value(val.m_Value) {}
		// Move constructor/operators seem unnecessary since it stores POD.
		template <typename A>
		constexpr fixed(arg_type<A> val) : m_Value(
			(sizeof(T) > sizeof(A)) ?
			T(((typename type_trait<A>::larger_type)(val)) << fractional_bits) :
			val << fractional_bits
		)
		{
			static_assert(type_trait<A>::is_integral, "constructor argument must be an integer or a float.");
		}
		constexpr fixed(arg_type<float> val) : m_Value(
			T((val * float(T(1) << fractional_bits)) + 0.5f)
		)
		{}

		constexpr fixed(arg_type<double> val) : fixed(float(val)) {}

		constexpr fixed & __restrict operator = (arg_type<fixed> val) __restrict
		{
			m_Value = val.m_Value;
			return *this;
		}

		template <typename A>
		constexpr fixed & __restrict operator = (arg_type<A> val) __restrict
		{
			static_assert(type_trait<A>::is_integral, "assignment argument must be an integer or a float.");
			if constexpr (sizeof(T) > sizeof(A))
			{
				using conv_t = typename type_trait<A>::larger_type;
				m_Value = conv_t(val) << fractional_bits;
				return *this;
			}
			else
			{
				m_Value = val << fractional_bits;
				return *this;
			}
		}

		constexpr fixed & __restrict operator = (arg_type<float> val) __restrict
		{
			constexpr const T multiplicand = T(1) << fractional_bits;
			constexpr const float multiplicandf = float(multiplicand);
			m_Value = T((val * multiplicandf) + 0.5f);

			return *this;
		}

		constexpr fixed & __restrict operator = (arg_type<double> val) __restrict
		{
			return *this = float(val);
		}

		constexpr operator float() const __restrict
		{
			return float(get_integer()) + (float(get_fraction()) / float(T{ 1 } << fractional_bits));
		}

		constexpr operator double() const __restrict
		{
			return float(*this);
		}

		constexpr operator T() const __restrict
		{
			constexpr const T fractional_mask = (T{ 1 } << fractional_bits) - 1;
			constexpr const uint8 half = 1_u8 << (fractional_bits - 1);
			return ((m_Value + half) & ~fractional_mask) >> fractional_bits;
		}

		constexpr bool operator == (arg_type<fixed> val) const __restrict
		{
			return m_Value == val.m_Value;
		}

		constexpr bool operator != (arg_type<fixed> val) const __restrict
		{
			return m_Value != val.m_Value;
		}

		constexpr bool operator > (arg_type<fixed> val) const __restrict
		{
			return m_Value > val.m_Value;
		}

		constexpr bool operator >= (arg_type<fixed> val) const __restrict
		{
			return m_Value >= val.m_Value;
		}

		constexpr bool operator < (arg_type<fixed> val) const __restrict
		{
			return m_Value < val.m_Value;
		}

		constexpr bool operator <= (arg_type<fixed> val) const __restrict
		{
			return m_Value <= val.m_Value;
		}

    constexpr bool operator == (arg_type<float> val) const __restrict
    {
      return m_Value == fixed(val);
    }

    constexpr bool operator != (arg_type<float> val) const __restrict
    {
      return m_Value != fixed(val);
    }

    constexpr bool operator > (arg_type<float> val) const __restrict
    {
      return m_Value > fixed(val);
    }

    constexpr bool operator >= (arg_type<float> val) const __restrict
    {
      return m_Value >= fixed(val);
    }

    constexpr bool operator < (arg_type<float> val) const __restrict
    {
      return m_Value < fixed(val);
    }

    constexpr bool operator <= (arg_type<float> val) const __restrict
    {
      return m_Value <= fixed(val);
    }

		constexpr bool operator == (arg_type<integer_t> val) const __restrict
		{
			return m_Value == T(T(val) << fractional_bits);
		}

		constexpr bool operator != (arg_type<integer_t> val) const __restrict
		{
			return m_Value != T(T(val) << fractional_bits);
		}

		constexpr bool operator > (arg_type<integer_t> val) const __restrict
		{
			return m_Value > T(T(val) << fractional_bits);
		}

		constexpr bool operator >= (arg_type<integer_t> val) const __restrict
		{
			return m_Value >= T(T(val) << fractional_bits);
		}

		constexpr bool operator < (arg_type<integer_t> val) const __restrict
		{
			return m_Value < T(T(val) << fractional_bits);
		}

		constexpr bool operator <= (arg_type<integer_t> val) const __restrict
		{
			return m_Value <= T(T(val) << fractional_bits );
		}

		constexpr fixed & __restrict operator *= (arg_type<fixed> val) __restrict
		{
			const T y = val.m_Value;
			const T x = m_Value;
			m_Value = bigint_t(bigint_t(x) * y) >> fractional_bits;
			return *this;
		}

		constexpr fixed & __restrict operator /= (arg_type<fixed> val) __restrict
		{
			const T y = val.m_Value;
			const T x = m_Value;

			m_Value = bigint_t(bigint_t(x) << fractional_bits) / y;
			return *this;
		}

		constexpr fixed & __restrict operator %= (arg_type<fixed> val) __restrict
		{
			const T y = val.m_Value;
			const T x = m_Value;

			m_Value = bigint_t(bigint_t(x) << fractional_bits) % y;
			return *this;
		}

		constexpr fixed & __restrict operator += (arg_type<fixed> val) __restrict
		{
			m_Value += val.m_Value;
			return *this;
		}

		constexpr fixed & __restrict operator -= (arg_type<fixed> val) __restrict
		{
			m_Value -= val.m_Value;
			return *this;
		}

		constexpr fixed operator * (arg_type<fixed> val) const __restrict
		{
			const T y = val.m_Value;
			const T x = m_Value;
			return from(bigint_t(bigint_t(x) * y) >> fractional_bits);
		}

		constexpr fixed operator / (arg_type<fixed> val) const __restrict
		{
			const T y = val.m_Value;
			const T x = m_Value;
			return from(bigint_t(bigint_t(x) << fractional_bits) / y);
		}

		constexpr fixed operator % (arg_type<fixed> val) const __restrict
		{
			const T y = val.m_Value;
			const T x = m_Value;
			return from(bigint_t(bigint_t(x) << fractional_bits) % y);
		}

		constexpr fixed operator + (arg_type<fixed> val) const __restrict
		{
			return from(m_Value + val.m_Value);
		}

		constexpr fixed operator - (arg_type<fixed> val) const __restrict
		{
			return from(m_Value - val.m_Value);
		}

		template <typename A>
		constexpr fixed & __restrict operator *= (arg_type<A> val) __restrict
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			m_Value *= val;
			return *this;
		}

		template <typename A>
		constexpr fixed & __restrict operator /= (arg_type<A> val) __restrict
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			m_Value /= val;
			return *this;
		}

		template <typename A>
		constexpr fixed & __restrict operator %= (arg_type<A> val) __restrict
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			m_Value %= val;
			return *this;
		}

		template <typename A>
		constexpr fixed & __restrict operator += (arg_type<A> val) __restrict
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			set_integer(get_integer() + val);
			return *this;
		}

		template <typename A>
		constexpr fixed & __restrict operator -= (arg_type<A> val) __restrict
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			set_integer(get_integer() - val);
			return *this;
		}

		template <typename A>
		constexpr fixed operator * (arg_type<A> val) const __restrict
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			return from(m_Value * val);
		}

		template <typename A>
		constexpr fixed operator / (arg_type<A> val) const __restrict
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			return from(m_Value / val);
		}

		template <typename A>
		constexpr fixed operator % (arg_type<A> val) const __restrict
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			return from(m_Value % val);
		}

		template <typename A>
		constexpr fixed operator + (arg_type<A> val) const __restrict
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			fixed ret{ from(m_Value) };
			ret.set_integer(ret.get_integer() + val);
			return ret;
		}

		template <typename A>
		constexpr fixed operator - (arg_type<A> val) const __restrict
		{
			static_assert(type_trait<A>::is_integral, "operator argument must be an integer or a float.");
			fixed ret{ from(m_Value) };
			ret.set_integer(ret.get_integer() - val);
			return ret;
		}

		constexpr fixed rounded() const __restrict
		{
			constexpr const T fractional_mask = (T{ 1 } << fractional_bits) - 1;
			constexpr const uintsz<(1 << fractional_bits) - 1> half = 1 << (fractional_bits - 1);
			return from(T(m_Value + half) & ~fractional_mask);
		}

		template <typename U>
		constexpr U rounded_to() const __restrict
		{
			constexpr const uintsz<(1 << fractional_bits) - 1> half = 1 << (fractional_bits - 1);
			return { T(m_Value + half) >> fractional_bits };
		}
	};

	template <uint64 value, uint8 minimum_frac, typename T>
	class _fixedsz final : trait::ce_only
	{
		static_assert(minimum_frac >= 1, "You must have greater than 0 fractional bits in a fixed-precision type");

		constexpr static auto typer()
		{
			if constexpr (is_same<T, void>)
			{
				using type = uintsz<value << minimum_frac>;
				return type{};
			}
			else
			{
				return T{};
			}
		}

		using TP = decltype(typer());

		static constexpr const uint8 bits_T = type_trait<TP>::bits;
		static constexpr const uint64 max_sz = type_trait<TP>::max >> 1; // fixed requires at least 1 bit of fractional precision
		static_assert(value <= max_sz, "Cannot fit the given value into a fixed-point type with the provided underlying type");
		static constexpr const uint8 integer_bits = ce_log2<value>;
		static constexpr const uint8 fractional_bits = bits_T - integer_bits;
		static_assert(fractional_bits >= minimum_frac, "Cannot fit the requested value into a fixed-precision type with the provided underlying type and requested minimum fractional bits");

	public:
		using type = fixed<TP, fractional_bits>;
	};

	template <uint64 value, uint8 minimum_frac = 1, typename T = void>
	using fixedsz = typename _fixedsz<value, minimum_frac, T>::type;
}
