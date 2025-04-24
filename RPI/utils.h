#pragma once

#include <concepts>
#include <numbers>

namespace wro
{
	template <typename T>
		requires std::convertible_to<T, float> && (std::integral<T> || std::floating_point<T>)
	constexpr bool approx(T a, T b, float tolerance = 0.1)
	{
		if constexpr (std::unsigned_integral<T>)
		{
			T max = std::max(a, b), min = std::min(a, b);
			return 2 * (max - min) <= tolerance * (static_cast<float>(max) + static_cast<float>(min));
		}
		else
		{
			return 2 * std::abs(a - b) <= tolerance * (static_cast<float>(a) + static_cast<float>(b));
		}
	}

	template <typename T>
		requires std::integral<T> || std::floating_point<T>
	struct Point
	{
		T x, y;

		Point() = default;
		Point(const T& x, const T& y)
			: x(x), y(y)
		{
		}
	};

	template <typename T>
		requires std::integral<T> || std::floating_point<T>
	double toRadians(T degrees)
	{
		return degrees * std::numbers::pi / 180;
	}

	template <typename T>
		requires std::integral<T> || std::floating_point<T>
	double toDegrees(T radians)
	{
		return radians * 180 / std::numbers::pi;
	}
}