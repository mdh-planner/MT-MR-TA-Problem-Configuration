#pragma once

// Copyright (C) by Tim V�lcker. All rights reserved.
// See the complete tutorial at www.timvoelcker.de/genetic_algorithm.html

// Functions for the creation of (pseudo) random numbers.
//
// A Mersenne Twister is used as a high quality (pseudo) random number
// generator. It is slower than simply calling the rand() function but it will 
// create better random numbers with a very good (uniform) distribution and
// is thread safe.
//
// A random number generator instance will be created once for every thread,
// so it must not be locked / synced, which will result in a better performance.
//
// The system time & thread id will be used to create an initial random seed.
// When the system clock is called by multiple threads at nearly the same time
// it might return the same value. A hash of the thread id will be combined
// with the current time to create a unique random seed for every thread.


#pragma once

#include <iostream>
#include <string>
#include <algorithm>
#include <iostream>
#include <list>
#include <numeric>
#include <vector>
#include <random>
using std::mt19937;
using std::uniform_int_distribution;
using std::uniform_real_distribution;
using std::bernoulli_distribution;

#include <chrono>

#include <thread>
using std::thread;

#include <functional>
using std::hash;

#include <memory>
using std::unique_ptr;


// By defining a fixed seed value the random number generator can be
// forced to produce the exact same set of pseudo random numbers all
// the time. This can be temporarily useful for general debugging or
// performance optimizations of algorithms.

#define USE_FIXED_SEED


inline unsigned int getRandomSeedF()
{
#ifdef USE_FIXED_SEED
	return 7612911124;
#else
	// Seed = Randomized variable address per thread (by os) + thread id + time

	const auto localVariable = 0;
	const auto randomMemoryAddress = reinterpret_cast<size_t>(&localVariable);
	const auto time = static_cast<size_t>(
		std::chrono::high_resolution_clock::now().time_since_epoch().count());
	const size_t thread_id = hash<thread::id>()(std::this_thread::get_id());
	unsigned int _returnSeed = static_cast<unsigned int>(randomMemoryAddress + thread_id + time);
	std::cout << "SEED: " << _returnSeed; std::cout << std::endl;
	return _returnSeed;
#endif
}


inline mt19937* getMersenneTwisterEngineF()
{
	static thread_local unique_ptr<mt19937> generator_owner;
	static thread_local mt19937* generator = nullptr;

	if (!generator)
	{
		generator_owner.reset(new mt19937(getRandomSeedF()));
		generator = generator_owner.get();
	}

	return generator;
}


template<typename T> T
inline getRandomIntegerInRangeF(T minInclusive, T maxInclusive)
{
	if (minInclusive > maxInclusive) {
		return -1;
	}
	auto generator = getMersenneTwisterEngineF();
	uniform_int_distribution<T> distribution(minInclusive, maxInclusive);
	return distribution(*generator);
}

template<typename T> T
inline getRandomRealInRangeF(T minInclusive, T maxInclusive) // T must be either float, double or long double. int wont work.
{
	if (minInclusive > maxInclusive) {
		return -1;
	}
	auto generator = getMersenneTwisterEngineF();
	uniform_real_distribution<T> distribution(minInclusive, maxInclusive);
	return distribution(*generator);

}


template<typename T> T
inline getRandomIntegerInRangeExcludingF(T minInclusive, T maxInclusive, T excluding)
{
	auto generator = getMersenneTwisterEngineF();

	if (maxInclusive == minInclusive) {
		if (excluding == 0) {
			return -1;
		}
		else {
			return 0;
		}

	}
	else if (minInclusive == excluding) {
		uniform_int_distribution<T> distribution(minInclusive + 1, maxInclusive);
		return distribution(*generator);
	}
	else if (maxInclusive == excluding) {
		uniform_int_distribution<T> distribution(minInclusive, maxInclusive - 1);
		return distribution(*generator);
	}

	else {
		int limit = (excluding - minInclusive) / (maxInclusive - minInclusive);
		auto realRnd = getRandomRealInRangeF(0.0, 1.0);

		if (realRnd > limit) {
			return getRandomIntegerInRangeF(excluding + 1, maxInclusive);
		}
		else {
			return getRandomIntegerInRangeF(minInclusive, excluding - 1);
		}

	}


}

template<typename T>
T getRandomIntVectorFromOtherVectorF(T &temp, T &_vector, int size, int task)
{

	//auto rng = getMersenneTwisterEngineF();
	//std::mt19937 rng(std::random_device{}());
	//std::shuffle(begin(temp), end(temp), rng);

	for (int i = 0; i < size; i++) {
		auto rnd = getRandomIntegerInRangeF(0, temp.size());

		_vector.emplace_back(temp[rnd]);


	}

	return _vector;
}

template<typename T>
T getRandomIntVectorInRangeF(T &v)
{
	auto rng = getMersenneTwisterEngineF();
	//std::mt19937 rng(std::random_device{}());
	std::iota(v.begin(), v.end(), v[0]);
	std::shuffle(v.begin(), v.end(), rng);

}

inline bool getRandomTrueWithProbabilityF(double probability)
{
	auto generator = getMersenneTwisterEngineF();
	std::bernoulli_distribution distribution(probability);
	return distribution(*generator);
}
#pragma once
