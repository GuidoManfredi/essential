/*
 * File: Random.cpp
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: April 2010
 * Description: manages pseudo-random numbers
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "Random.h"
#include "Timestamp.h"
#include <cstdlib>
using namespace std;

bool Random::m_seeded_current_time = false;
bool Random::m_seeded_int = false;

void Random::SeedRand(){
	Timestamp time;
	time.setToCurrentTime();
	srand((unsigned)time.getFloatTime());
}

void Random::SeedRandOnce()
{
  if(!m_seeded_current_time)
  {
    Random::SeedRand();
    m_seeded_current_time = true;
  }
}

void Random::SeedRand(int seed)
{
	srand(seed);
}

void Random::SeedRandOnce(int seed)
{
  if(!m_seeded_int)
  {
    Random::SeedRand(seed);
    m_seeded_int = true;
  }
}

int Random::RandomInt(int min, int max){
	int d = max - min + 1;
	return int(((double)rand()/((double)RAND_MAX + 1.0)) * d) + min;
}


