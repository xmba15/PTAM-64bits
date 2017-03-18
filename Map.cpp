// Copyright 2008 Isis Innovation Limited
#include "Map.h"
#include "MapPoint.h"

Map::Map()
{
	Reset();
}

void Map::Reset()
{
	vpPoints.clear();
	bGood = false;
	EmptyTrash();
}

void Map::MoveBadPointsToTrash()
{
	int nBad = 0;
	for (int i = vpPoints.size() - 1; i >= 0; i--)
	{
		if (vpPoints[i].use_count() == 0) {
			vpPoints.erase(vpPoints.begin() + i);
			continue;
		}
		if (vpPoints[i]->bBad)
		{
			vpPointsTrash.push_back(vpPoints[i]);
			vpPoints.erase(vpPoints.begin() + i);
			nBad++;
		}
	}
}

void Map::EmptyTrash()
{
	vpPointsTrash.clear();
}




