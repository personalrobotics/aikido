#include <r3/Skeleton.h>
#include <iostream>

using namespace ::r3;

Skeleton::Skeleton()
{
    std::cout << "+Skeleton(" << this << ")" << std::endl;
}

Skeleton::~Skeleton()
{
    std::cout << "-Skeleton(" << this << ")" << std::endl;
}

std::string Skeleton::name() const
{
    return "Chris Dellin";
}
