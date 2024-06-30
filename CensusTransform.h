#pragma once
#define __builtin_popcount __popcnt 

#define __builtin_popcountl __popcnt
//#include "SemiGlobalMatcher.h"

inline uint32 get_census_value_5x5(const uint8* image, int col, int row, const sint32& cols, const sint32& rows);
inline uint64 get_census_value_ternary_5x5(const uint8* image, int col, int row, const sint32& cols, const sint32& rows, int diff_threshold=2);
inline uint64 get_census_value_improvet_5x5(const uint8* image, int col, int row, const sint32& cols, const sint32& rows);
inline uint64 get_census_value_improvet3_5x5(const uint8* image, int col, int row, const sint32& cols, const sint32& rows);



uint32  get_census_value_5x5(const uint8* image, int col, int row, const sint32& cols, const sint32& rows) {
	// This will be a 24 bit sequence.
	uint32 output = 0;
	uint32 addend = 1;
	const uint8 center = image[row * cols + col];
	for (int r = row + 2; r >= row - 2; --r) {
		for (int c = col + 2; c >= col - 2; --c) {
			if ((r == row) && (c == col)) // Skip the central pixel
				continue;
			if (image[r*cols + c] > center)
				output += addend;
			addend *= 2;
		}
	}
	return output;
}



uint64 get_census_value_ternary_5x5(const uint8* image, int col, int row, const sint32& cols, const sint32& rows, int diff_threshold) {
	// This will be a 48 bit sequence.
	uint64 output = 0;
	uint64 addend = 1;
	const int center = image[row * cols + col];
	const int low_thresh = center - diff_threshold;
	const int high_thresh = center + diff_threshold;
	for (int r = row + 2; r >= row - 2; --r) {
		for (int c = col + 2; c >= col - 2; --c) {
			if ((r == row) && (c == col)) // Skip the central pixel
				continue;
			int val = image[r*cols + c];
			if (val >= low_thresh) {
				output += addend;
				if (val > high_thresh) // Greater, += 11
					output += addend * 2;
				// else Middle range, += 01
			}
			// For low range, += 00

			addend *= 4;
		}
	}
	return output;
}




inline uint8 hamming_distance32(const uint32& x, const uint32& y)
{
	uint32 dist = 0, val = x ^ y;
	return __builtin_popcount(val);
	// Count the number of set bits
	/*while (val) {
		++dist;
		val &= val - 1;
	}

	return static_cast<uint8>(dist);*/
}

inline uint8 hamming_distance64(const uint64& x, const uint64& y)
{
	uint64 dist = 0, val = x ^ y;

	// Count the number of set bits
	while (val) {
		++dist;
		val &= val - 1;
	}

	return static_cast<uint8>(dist);
}

uint64 get_census_value_improvet_5x5(const uint8* image, int col, int row, const sint32& cols, const sint32& rows) {
	// This will be a 48 bit sequence.四个方向取均值，设定两个阈值
	uint64 output = 0;
	uint64 addend = 1;
	const int center = image[row * cols + col];
	double Imleft = (image[row * cols + col] + image[row * cols + col - 1] + image[row * cols + col - 2]) / 3;
	double Imright = (image[row * cols + col] + image[row * cols + col + 1] + image[row * cols + col + 2]) / 3;
	double Imup = (image[row * cols + col] + image[(row-1) * cols + col] + image[(row-2) * cols + col]) / 3;
	double Imdown = (image[row * cols + col] + image[(row + 1) * cols + col] + image[(row + 2) * cols + col]) / 3;
	double Imupleft = (image[row * cols + col] + image[(row - 1) * cols + col-1] + image[(row - 2) * cols + col-2]) / 3;
	double Imupright = (image[row * cols + col] + image[(row + 1) * cols + col - 1] + image[(row + 2) * cols + col - 2]) / 3;
	double Imdownleft = (image[row * cols + col] + image[(row + 1) * cols + col - 1] + image[(row + 2) * cols + col - 2]) / 3;
	double Imdownright = (image[row * cols + col] + image[(row + 1) * cols + col + 1] + image[(row + 2) * cols + col + 2]) / 3;
	int remain_sum_8 = image[(row - 2) * cols + col - 1] + image[(row - 2) * cols + col + 1] + image[(row - 1) * cols + col - 2] + image[(row - 1) * cols + col + 2] + image[(row + 1) * cols + col - 2] + image[(row + 1) * cols + col + 2] + image[(row + 2) * cols + col - 1] + image[(row + 2) * cols + col + 1];
	double Imean = ((Imleft + Imright + Imup + Imdown + Imupleft + Imupright + Imdownleft + Imdownright) * 3 - 7 * image[row * cols + col] + remain_sum_8) / 25;
	const int low_thresh = std::min({Imleft, Imright, Imup, Imdown, Imean});
	const int high_thresh = std::max({ Imleft, Imright, Imup, Imdown, Imean });
	for (int r = row + 2; r >= row - 2; --r) {
		for (int c = col + 2; c >= col - 2; --c) {
			if ((r == row) && (c == col)) // Skip the central pixel
				continue;
			int val = image[r * cols + c];
			if (val >= low_thresh) {
				output += addend;
				if (val > high_thresh) // Greater, += 11
					output += addend * 2;
				// else Middle range, += 01
			}
			// For low range, += 00

			addend *= 4;
		}
	}	
	return output;
}


uint64 get_census_value_improvet3_5x5(const uint8* image, int col, int row, const sint32& cols, const sint32& rows) {
	// This will be a 48 bit sequence.四个方向取均值，设定两个阈值，每个方向上进行加权获得每个方向的阈值
	uint64 output = 0;
	uint64 addend = 1;
	const int center = image[row * cols + col];
	double Imleft = 0.7*image[row * cols + col] + 0.2*image[row * cols + col - 1] + 0.1*image[row * cols + col - 2];
	double Imright = 0.7*image[row * cols + col] + 0.2*image[row * cols + col + 1] + 0.1*image[row * cols + col + 2];
	double Imup = 0.7*image[row * cols + col] + 0.2*image[(row - 1) * cols + col] + 0.1*image[(row - 2) * cols + col];
	double Imdown = 0.7*image[row * cols + col] + 0.2*image[(row + 1) * cols + col] + 0.1*image[(row + 2) * cols + col];
	double Imupleft = 0.5*image[row * cols + col] + 0.3*image[(row - 1) * cols + col - 1] + 0.2*image[(row - 2) * cols + col - 2];
	double Imupright = 0.5*image[row * cols + col] + 0.3*image[(row + 1) * cols + col - 1] + 0.2*image[(row + 2) * cols + col - 2];
	double Imdownleft = 0.5*image[row * cols + col] + 0.3*image[(row + 1) * cols + col - 1] + 0.2*image[(row + 2) * cols + col - 2];
	double Imdownright = 0.5*image[row * cols + col] + 0.3*image[(row + 1) * cols + col + 1] + 0.2*image[(row + 2) * cols + col + 2];
	int remain_sum_8 = image[(row - 2) * cols + col - 1] + image[(row - 2) * cols + col + 1] + image[(row - 1) * cols + col - 2] + image[(row - 1) * cols + col + 2] + image[(row + 1) * cols + col - 2] + image[(row + 1) * cols + col + 2] + image[(row + 2) * cols + col - 1] + image[(row + 2) * cols + col + 1];
	double Imean = ((Imleft + Imright + Imup + Imdown + Imupleft + Imupright + Imdownleft + Imdownright) * 3 - 7 * image[row * cols + col] + remain_sum_8) / 25;
	/*const int low_thresh = std::min({ Imleft, Imright, Imup, Imdown, Imupleft, Imupright, Imdownleft, Imdownright });
	const int high_thresh = std::max({ Imleft, Imright, Imup, Imdown, Imupleft, Imupright, Imdownleft, Imdownright });*/
	const int low_thresh = std::min({ Imleft, Imright, Imup, Imdown });
	const int high_thresh = std::max({ Imleft, Imright, Imup, Imdown });
	for (int r = row + 2; r >= row - 2; --r) {
		for (int c = col + 2; c >= col - 2; --c) {
			if ((r == row) && (c == col)) // Skip the central pixel
				continue;
			int val = image[r * cols + c];
			if (val >= low_thresh) {
				output += addend;
				if (val > high_thresh) // Greater, += 11
					output += addend * 2;
				// else Middle range, += 01
			}
			// For low range, += 00

			addend *= 4;
		}
	}
	return output;
}

