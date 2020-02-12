// PX2RobotDatas.cpp

#include "PX2RobotDatas.hpp"
using namespace PX2;

//----------------------------------------------------------------------------
RobotMapDataStruct::RobotMapDataStruct()
{
	MapWidth = 256;
	MapHeight = 256;
	MapResolution = 0.1f;
	IndexX = 0;
	IndexY = 0;
	CurAngle = 0.0f;
}
//----------------------------------------------------------------------------
RobotMapDataStruct::~RobotMapDataStruct()
{
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
RobotToDoStruct::RobotToDoStruct()
{
	LeftSpeed = 0.0f;
	RightSpeed = 0.0f;
	HeadDegree = 0;

	Type = 0;
	Range = 0.0f;
	Value = 0;
}
//----------------------------------------------------------------------------
RobotToDoStruct::~RobotToDoStruct()
{

}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
PX2_IMPLEMENT_RTTI(PX2, Object, RobotMapData);
PX2_IMPLEMENT_STREAM(RobotMapData);
PX2_IMPLEMENT_FACTORY(RobotMapData);
PX2_IMPLEMENT_DEFAULT_NAMES(Object, RobotMapData);
//----------------------------------------------------------------------------
RobotMapData::RobotMapData()
{
}
//----------------------------------------------------------------------------
RobotMapData::~RobotMapData()
{
}
//----------------------------------------------------------------------------
std::vector<unsigned char> RobotMapData::_ExtendLarger(
	const std::vector<unsigned char> &mapping, int mapWidth,
	int mapHeight, int equalValue)
{
	std::vector<unsigned char> retMapping = mapping;

	int x = 0;
	int y = 0;
	int mapVal = 200;
	int index = 0;
	for (int y = 0; y < mapHeight; y++)
	{
		for (int x = 0; x < mapWidth; x++)
		{
			index = y * mapWidth + x;

			mapVal = mapping[index];

			bool isHasObst = false;
			if (equalValue == mapVal)
			{
				isHasObst = true;
			}

			if (isHasObst)
			{
				_SetMapUsingData(retMapping, mapWidth, mapHeight, x, y, equalValue);

				// left
				int left = x - 1;
				if (left >= 0)
				{
					_SetMapUsingData(retMapping, mapWidth, mapHeight, left, y, equalValue);
				}

				left = x - 2;
				if (left >= 0)
				{
					_SetMapUsingData(retMapping, mapWidth, mapHeight, left, y, equalValue);
				}

				// right
				int right = x + 1;
				if (right < MapStruct.MapWidth)
				{
					_SetMapUsingData(retMapping, mapWidth, mapHeight, right, y, equalValue);
				}

				right = x + 2;
				if (right < MapStruct.MapWidth)
				{
					_SetMapUsingData(retMapping, mapWidth, mapHeight, right, y, equalValue);
				}

				// down
				int down = y - 1;
				if (down >= 0)
				{
					_SetMapUsingData(retMapping, mapWidth, mapHeight, x, down, equalValue);
				}

				down = y - 2;
				if (down >= 0)
				{
					_SetMapUsingData(retMapping, mapWidth, mapHeight, x, down, equalValue);
				}

				// up
				int up = y + 1;
				if (up < MapStruct.MapHeight)
				{
					_SetMapUsingData(retMapping, mapWidth, mapHeight, x, up, equalValue);
				}

				up = y + 2;
				if (up < MapStruct.MapHeight)
				{
					_SetMapUsingData(retMapping, mapWidth, mapHeight, x, up, equalValue);
				}
			}
		}
	}

	return retMapping;
}
//----------------------------------------------------------------------------
void RobotMapData::Initlize(int mapWidth, int mapHeight, float resolution)
{
	int mapAllSize = mapWidth * mapHeight;

	MapStruct.MapWidth = mapWidth;
	MapStruct.MapHeight = mapHeight;
	MapStruct.MapResolution = resolution;

	Map2DOrigin.resize(mapAllSize);
	memset(&(Map2DOrigin[0]), RobotMapData::MOT_NONE, (int)mapAllSize);

	SelfDrawMapData2D.resize(mapAllSize);
	memset(&(SelfDrawMapData2D[0]), RobotMapData::MOT_NONE, (int)mapAllSize);

	PathFinderLineDraw.resize(mapAllSize);
	memset(&(PathFinderLineDraw[0]), 0, (int)mapAllSize);
}
//----------------------------------------------------------------------------
void RobotMapData::_SetMapUsingData(std::vector<unsigned char> &maping,
	int mapWidth, int mapHeight, int x, int y, int val)
{
	int index = y*mapWidth + x;
	maping[index] = (unsigned char)val;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// ³Ö¾Ã»¯
//----------------------------------------------------------------------------
RobotMapData::RobotMapData(LoadConstructor value) :
Object(value)
{
}
//----------------------------------------------------------------------------
void RobotMapData::Load(InStream& source)
{
	PX2_BEGIN_DEBUG_STREAM_LOAD(source);

	Object::Load(source);
	PX2_VERSION_LOAD(source);

	source.Read(MapStruct.MapWidth);
	source.Read(MapStruct.MapHeight);
	source.Read(MapStruct.MapResolution);
	source.Read(MapStruct.IndexX);
	source.Read(MapStruct.IndexY);
	source.ReadAggregate(MapStruct.CurPos);
	source.Read(MapStruct.CurAngle);

	int numDatas = 0;
	source.Read(numDatas);

	Map2DOrigin.resize(numDatas);
	for (int i = 0; i < numDatas; i++)
	{
		source.Read<uint8_t>(Map2DOrigin[i]);
	}

	SelfDrawMapData2D.resize(numDatas);
	for (int i = 0; i < numDatas; i++)
	{
		source.Read<uint8_t>(SelfDrawMapData2D[i]);
	}

	PathFinderLineDraw.resize(numDatas);
	for (int i = 0; i < numDatas; i++)
	{
		source.Read<uint8_t>(PathFinderLineDraw[i]);
	}
	memset(&PathFinderLineDraw[0], 0, numDatas);

	PX2_END_DEBUG_STREAM_LOAD(RobotMapData, source);
}
//----------------------------------------------------------------------------
void RobotMapData::Link(InStream& source)
{
	Object::Link(source);
}
//----------------------------------------------------------------------------
void RobotMapData::PostLink()
{
	Object::PostLink();
}
//----------------------------------------------------------------------------
bool RobotMapData::Register(OutStream& target) const
{
	if (Object::Register(target))
	{
		return true;
	}

	return false;
}
//----------------------------------------------------------------------------
void RobotMapData::Save(OutStream& target) const
{
	PX2_BEGIN_DEBUG_STREAM_SAVE(target);

	Object::Save(target);
	PX2_VERSION_SAVE(target);

	target.Write(MapStruct.MapWidth);
	target.Write(MapStruct.MapHeight);
	target.Write(MapStruct.MapResolution);
	target.Write(MapStruct.IndexX);
	target.Write(MapStruct.IndexY);
	target.WriteAggregate(MapStruct.CurPos);
	target.Write(MapStruct.CurAngle);

	int numDatas = (int)Map2DOrigin.size();
	target.Write(numDatas);
	for (int i = 0; i < numDatas; i++)
	{
		target.Write<uint8_t>(Map2DOrigin[i]);
	}
	for (int i = 0; i < numDatas; i++)
	{
		target.Write<uint8_t>(SelfDrawMapData2D[i]);
	}
	for (int i = 0; i < numDatas; i++)
	{
		target.Write<uint8_t>(PathFinderLineDraw[i]);
	}

	PX2_END_DEBUG_STREAM_SAVE(RobotMapData, target);
}
//----------------------------------------------------------------------------
int RobotMapData::GetStreamingSize(Stream &stream) const
{
	int size = Object::GetStreamingSize(stream);
	size += PX2_VERSION_SIZE(mVersion);

	size += sizeof(MapStruct.MapWidth);
	size += sizeof(MapStruct.MapHeight);
	size += sizeof(MapStruct.MapResolution);
	size += sizeof(MapStruct.IndexX);
	size += sizeof(MapStruct.IndexY);
	size += sizeof(MapStruct.CurPos);
	size += sizeof(MapStruct.CurAngle);

	int numDatas = (int)Map2DOrigin.size();
	size += sizeof(numDatas);
	size += numDatas * sizeof(uint8_t);
	size += numDatas * sizeof(uint8_t);
	size += numDatas * sizeof(uint8_t);

	return size;
}
//----------------------------------------------------------------------------
