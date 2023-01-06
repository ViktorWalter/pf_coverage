#pragma once

// STL
#include <vector>

class Triangulation
{
public:
    explicit Triangulation(std::vector<std::vector<std::size_t>> neighbors) : mNeighbors(std::move(neighbors))
    {}

    std::size_t getNbVertices() const
    {
        return mNeighbors.size();
    }

    const std::vector<std::size_t>& getNeighbors(std::size_t i) const
    {
        return mNeighbors[i];
    }

private:
    std::vector<std::vector<std::size_t>> mNeighbors;
};
