#include "Util.h"

uint32_t lcm(uint32_t n1, uint32_t n2)
{
  uint32_t maxN = (n1 > n2) ? n1 : n2;

  while (true)
  {
    if (maxN % n1 == 0 && maxN % n2 == 0)
    {
      break;
    }
    maxN++;
  }

  return maxN;
}

uint32_t gcd(uint32_t n1, uint32_t n2)
{
  while (n1 != n2)
  {
    if (n1 > n2)
      n1 -= n2;
    else
      n2 -= n1;
  }

  return n1;
}
