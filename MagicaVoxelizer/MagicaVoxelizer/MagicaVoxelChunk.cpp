// Sample.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include <fstream>
#include "igl/signed_distance.h"
#include "MagicaVoxelizer.h"

// see https://github.com/ephtracy/voxel-model/blob/master/MagicaVoxel-file-format-vox.txt
	////
	// implementation
	////

const std::map<std::string, MagicaVoxelChunk::chunkType> MagicaVoxelChunk::strToType({
	{std::string("MAIN",4), chunkType::MAIN},
	{std::string("PACK",4), chunkType::PACK},
	{std::string("SIZE",4), chunkType::SIZE},
	{std::string("XYZI",4), chunkType::XYZI},
	{std::string("RGBA",4), chunkType::RGBA},
	{std::string("MATT",4), chunkType::MATT},
	{std::string("nTRN",4), chunkType::nTRN},
	{std::string("nGRP",4), chunkType::nGRP},
	{std::string("nSHP",4), chunkType::nSHP},
	{std::string("MATL",4), chunkType::MATL},
	{std::string("LAYR",4), chunkType::LAYR}
	});

const std::map<MagicaVoxelChunk::chunkType, std::string> MagicaVoxelChunk::typeToStr({
	{chunkType::MAIN, std::string("MAIN",4)},
	{chunkType::PACK, std::string("PACK",4)},
	{chunkType::SIZE, std::string("SIZE",4)},
	{chunkType::XYZI, std::string("XYZI",4)},
	{chunkType::RGBA, std::string("RGBA",4)},
	{chunkType::MATT, std::string("MATT",4)},
	{chunkType::nTRN, std::string("nTRN",4)},
	{chunkType::nGRP, std::string("nGRP",4)},
	{chunkType::nSHP, std::string("nSHP",4)},
	{chunkType::MATL, std::string("MATL",4)},
	{chunkType::LAYR, std::string("LAYR",4)}
	});

// default palette is changed??
const std::vector<unsigned int> MagicaVoxelChunk::default_palette({
	0xffffffff, 0xffccffff, 0xff99ffff, 0xff66ffff, 0xff33ffff, 0xff00ffff,
	0xffffccff, 0xffccccff, 0xff99ccff, 0xff66ccff, 0xff33ccff, 0xff00ccff,
	0xffff99ff, 0xffcc99ff, 0xff9999ff,	0xff6699ff, 0xff3399ff, 0xff0099ff,
	0xffff66ff, 0xffcc66ff, 0xff9966ff, 0xff6666ff, 0xff3366ff, 0xff0066ff,
	0xffff33ff, 0xffcc33ff, 0xff9933ff, 0xff6633ff, 0xff3333ff, 0xff0033ff,
	0xffff00ff,	0xffcc00ff, 0xff9900ff, 0xff6600ff, 0xff3300ff, 0xff0000ff,

	0xffffffcc, 0xffccffcc, 0xff99ffcc, 0xff66ffcc, 0xff33ffcc, 0xff00ffcc,
	0xffffcccc, 0xffcccccc, 0xff99cccc, 0xff66cccc, 0xff33cccc,	0xff00cccc,
	0xffff99cc, 0xffcc99cc, 0xff9999cc, 0xff6699cc, 0xff3399cc, 0xff0099cc,
	0xffff66cc, 0xffcc66cc, 0xff9966cc, 0xff6666cc, 0xff3366cc, 0xff0066cc,
	0xffff33cc, 0xffcc33cc, 0xff9933cc,	0xff6633cc, 0xff3333cc, 0xff0033cc,
	0xffff00cc, 0xffcc00cc, 0xff9900cc, 0xff6600cc, 0xff3300cc, 0xff0000cc,

	0xffffff99, 0xffccff99, 0xff99ff99, 0xff66ff99, 0xff33ff99, 0xff00ff99,
	0xffffcc99,	0xffcccc99, 0xff99cc99, 0xff66cc99, 0xff33cc99, 0xff00cc99,
	0xffff9999, 0xffcc9999, 0xff999999, 0xff669999, 0xff339999, 0xff009999,
	0xffff6699, 0xffcc6699, 0xff996699, 0xff666699, 0xff336699,	0xff006699,
	0xffff3399, 0xffcc3399, 0xff993399, 0xff663399, 0xff333399, 0xff003399,
	0xffff0099, 0xffcc0099, 0xff990099, 0xff660099, 0xff330099, 0xff000099,

	0xffffff66, 0xffccff66, 0xff99ff66,	0xff66ff66, 0xff33ff66, 0xff00ff66,
	0xffffcc66, 0xffcccc66, 0xff99cc66, 0xff66cc66, 0xff33cc66, 0xff00cc66,
	0xffff9966, 0xffcc9966, 0xff999966, 0xff669966, 0xff339966, 0xff009966,
	0xffff6666,	0xffcc6666, 0xff996666, 0xff666666, 0xff336666, 0xff006666,
	0xffff3366, 0xffcc3366, 0xff993366, 0xff663366, 0xff333366, 0xff003366,
	0xffff0066, 0xffcc0066, 0xff990066, 0xff660066, 0xff330066,	0xff000066,

	0xffffff33, 0xffccff33, 0xff99ff33, 0xff66ff33, 0xff33ff33, 0xff00ff33,
	0xffffcc33, 0xffcccc33, 0xff99cc33, 0xff66cc33, 0xff33cc33, 0xff00cc33,
	0xffff9933, 0xffcc9933, 0xff999933,	0xff669933, 0xff339933, 0xff009933,
	0xffff6633, 0xffcc6633, 0xff996633, 0xff666633, 0xff336633, 0xff006633,
	0xffff3333, 0xffcc3333, 0xff993333, 0xff663333, 0xff333333, 0xff003333,
	0xffff0033,	0xffcc0033, 0xff990033, 0xff660033, 0xff330033, 0xff000033,

	0xffffff00, 0xffccff00, 0xff99ff00, 0xff66ff00, 0xff33ff00, 0xff00ff00,
	0xffffcc00, 0xffcccc00, 0xff99cc00, 0xff66cc00, 0xff33cc00,	0xff00cc00,
	0xffff9900, 0xffcc9900, 0xff999900, 0xff669900, 0xff339900, 0xff009900,
	0xffff6600, 0xffcc6600, 0xff996600, 0xff666600, 0xff336600, 0xff006600,
	0xffff3300, 0xffcc3300, 0xff993300,	0xff663300, 0xff333300, 0xff003300,
	0xffff0000, 0xffcc0000, 0xff990000, 0xff660000, 0xff330000,

	0xff0000ee, 0xff0000dd, 0xff0000bb, 0xff0000aa, 0xff000088, 0xff000077,
	0xff000055, 0xff000044,	0xff000022, 0xff000011, 0xff00ee00, 0xff00dd00,
	0xff00bb00, 0xff00aa00, 0xff008800, 0xff007700, 0xff005500, 0xff004400,
	0xff002200, 0xff001100, 0xffee0000, 0xffdd0000, 0xffbb0000, 0xffaa0000,
	0xff880000, 0xff770000, 0xff550000, 0xff440000, 0xff220000, 0xff110000,
	0xffeeeeee, 0xffdddddd, 0xffbbbbbb, 0xffaaaaaa, 0xff888888, 0xff777777,
	0xff555555, 0xff444444, 0xff222222, 0xff111111, 0x00000000
	});


MagicaVoxelChunk::MagicaVoxelChunk(const std::string& typeStr)
{
	type = strToType.at(typeStr);
	switch (type)
	{
	case chunkType::MAIN:
		content.resize(0);
		matContent.resize(0);
		break;
	case chunkType::PACK:
		content.resize(1);
		matContent.resize(0);
		break;
	case chunkType::SIZE:
		content.resize(3);
		matContent.resize(0);
		break;
	case chunkType::XYZI:
		content.resize(1); // need to be resized afterward
		matContent.resize(0);
		break;
	case chunkType::RGBA:
		content.resize(256);
		matContent.resize(0);
		break;
	case chunkType::MATT:
		content.resize(3);
		matContent.resize(1);
		break;
	default:
		break;
	}
}

MagicaVoxelChunk::~MagicaVoxelChunk()
{
}

void MagicaVoxelChunk::appendChild(const MagicaVoxelChunk& child)
{
	if (type == chunkType::MAIN)
	{
		children.push_back(child);
	}
}

int MagicaVoxelChunk::sizeOfChunk(const bool recursive) const
{
	// chunkID + contentSize + childrenSize + itsContent
	int size = sizeof(char) * 4 + sizeof(int) + sizeof(int) + sizeof(int)*content.size() + sizeof(float)*matContent.size();
	if (recursive)
	{
		for (const auto& c : children)
			size += (c.sizeOfChunk(true));
	}
	return size;
}

int MagicaVoxelChunk::writeChunkToFile(std::ofstream& file) const
{
	int byteWritten = 0;
	bool success = true;
	try
	{
		// chunk id
		file.write(typeToStr.at(type).c_str(), sizeof(unsigned char) * 4);
		byteWritten += sizeof(unsigned char) * 4;

		// byte of content
		const int byteOfContent = sizeof(int)*content.size() + sizeof(float)*matContent.size();
		file.write((const char*)&byteOfContent, sizeof(int) * 1);
		byteWritten += sizeof(int) * 1;

		// byte of children chunks
		const int byteOfChildren = sizeOfChunk(true) - (sizeof(char) * 4 + sizeof(int) + sizeof(int) + sizeof(int)*content.size() + sizeof(float)*matContent.size());
		file.write((const char*)&byteOfChildren, sizeof(int) * 1);
		byteWritten += sizeof(int) * 1;

		// content
		switch (type)
		{
		case chunkType::MAIN:
			// no content
			break;
		case chunkType::PACK:
		case chunkType::SIZE:
		case chunkType::XYZI:
		case chunkType::RGBA:
			file.write((const char*)&content[0], sizeof(int) * content.size());
			byteWritten += sizeof(int) * content.size();
			// content.size() ==
			// PACK: 1
			// SIZE: 3
			// XYZI: (1+N)
			// RGBA: 256
			// matContent.size() == 0
			break;
		case chunkType::MATT:
			file.write((const char*)&content[0], sizeof(int) * 2);
			byteWritten += sizeof(int) * 2;
			file.write((const char*)&matContent[0], sizeof(float) * 1);
			byteWritten += sizeof(float) * 1;
			file.write((const char*)&content[2], sizeof(int) * 1);
			byteWritten += sizeof(int) * 1;
			file.write((const char*)&matContent[1], sizeof(float) * (matContent.size() - 1));
			byteWritten += sizeof(float) * (matContent.size() - 1);
			// content.size() == 3
			// matContent.size() == (1+N)
			break;
		default:
			break;
		}

	}
	catch (int e) {
		return false;
	}

	// children
	for (const auto& c : children)
	{
		byteWritten += c.writeChunkToFile(file);
		if (!success) {
			return -1;
		}
	}
	return byteWritten;
}

bool MagicaVoxelChunk::writeVoxelToFile(const std::string& fileName) const
{
	if (type == chunkType::MAIN)
	{
		std::ofstream file;
		file.open(fileName, std::ios::out | std::ios::binary | std::ios::trunc);
		if (!file)
		{
			std::cerr << "cannot open file." << std::endl;
			return false;
		}

		// write header
		int version = 150;
		file.write("VOX ", sizeof(unsigned char) * 4);
		file.write((const char*)&version, sizeof(int));
		int writeByte = writeChunkToFile(file);
		file.close();

		return (writeByte > 0);
	}
	else
	{
		std::cerr << "writeToFile(const std::string&) const must be called with MAIN chunk." << std::endl;
		return false;
	}
}

bool MagicaVoxelChunk::readVoxelFromFile(const std::string& fileName) {
	if (type != chunkType::MAIN)
	{
		std::cerr << "this function is valid only for MAIN chunk." << std::endl;
	}
	try {
		std::ifstream file;
		file.open(fileName, std::ios::in | std::ios::binary);
		if (!file)
		{
			std::cerr << "cannot open file." << std::endl;
			return false;
		}

		char magic[4];
		int version;
		file.read(magic, 4);
		if (strncmp(magic, "VOX ", 4) != 0)
		{
			std::cerr << "invalid magic (VOX ) ..." << std::endl;
			return false;
		}
		file.read((char*)&version, sizeof(int));
		if (version != 150)
		{
			std::cerr << "invalid version (150) ..." << std::endl;
			return false;
		}

		{
			char chunkId[4];
			int chunkContentBytes;
			int chunkChildrenBytes;
			file.read(chunkId, 4);
			if (strncmp(chunkId, "MAIN", 4) != 0)
			{
				std::cerr << "invalid id (MAIN) ..." << std::endl;
				return false;
			}
			file.read((char*)&chunkContentBytes, sizeof(int));
			if (chunkContentBytes != 0)
			{
				std::cerr << "invalid content size (MAIN, 0) ..." << std::endl;
				return false;
			}
			file.read((char*)&chunkChildrenBytes, sizeof(int));
			// currently ignored.

			// no content
		}

		char chunkId[4];
		while (file.read(chunkId, 4))
		{
			int chunkContentBytes;
			int chunkChildrenBytes;

			union converter {
				unsigned char c[4];
				int i;
				float f;
			};
			union converter conv;

			file.read((char*)&chunkContentBytes, sizeof(int));
			file.read((char*)&chunkChildrenBytes, sizeof(int));
			std::vector<unsigned char> content(chunkContentBytes);
			file.read((char*)&content[0], sizeof(unsigned char)*chunkContentBytes);

			chunkType chunktype = (strToType.find(std::string(chunkId, 4)) == strToType.end()) ? (chunkType::NONE) : (strToType.at(std::string(chunkId, 4)));
			switch (chunktype)
			{
			case chunkType::PACK:
			{
				MagicaVoxelChunk packChunk(std::string("PACK", 4));
				packChunk.content.resize(chunkContentBytes / sizeof(int));
				memcpy(conv.c, content.data(), sizeof(int));
				packChunk.content.at(0) = conv.i;
				appendChild(packChunk);
				break;
			}
			case chunkType::SIZE:
			{
				MagicaVoxelChunk sizeChunk(std::string("SIZE", 4));
				sizeChunk.content.resize(chunkContentBytes / sizeof(int));
				memcpy(conv.c, content.data(), sizeof(int));
				sizeChunk.content.at(0) = conv.i;
				memcpy(conv.c, content.data() + sizeof(int), sizeof(int));
				sizeChunk.content.at(1) = conv.i;
				memcpy(conv.c, content.data() + sizeof(int) * 2, sizeof(int));
				sizeChunk.content.at(2) = conv.i;
				appendChild(sizeChunk);
				break;
			}
			case chunkType::XYZI:
			{
				MagicaVoxelChunk xyziChunk(std::string("XYZI", 4));
				xyziChunk.content.resize(chunkContentBytes / sizeof(int));
				memcpy(conv.c, content.data(), sizeof(int));
				xyziChunk.content.at(0) = conv.i;
				for (int v = 1; v < xyziChunk.content.at(0) + 1; ++v)
				{
					memcpy(conv.c, content.data() + sizeof(int) * v, sizeof(int));
					xyziChunk.content.at(v) = conv.i;
				}
				appendChild(xyziChunk);
				break;
			}
			case chunkType::RGBA:
			{
				MagicaVoxelChunk rgbaChunk(std::string("RGBA", 4));
				rgbaChunk.content.resize(chunkContentBytes / sizeof(int));
				for (int rgba = 0; rgba < 256; ++rgba)
				{
					memcpy(conv.c, content.data() + sizeof(int) * rgba, sizeof(int));
					rgbaChunk.content.at(rgba) = conv.i;
				}
				appendChild(rgbaChunk);
				break;
			}
			case chunkType::MATT:
			{
				MagicaVoxelChunk mattChunk(std::string("MATT", 4));
				mattChunk.content.resize(3);
				mattChunk.matContent.resize(chunkContentBytes / sizeof(int) - 3);
				memcpy(conv.c, content.data() + sizeof(int) * 0, sizeof(int));
				mattChunk.content.at(0) = conv.i;
				memcpy(conv.c, content.data() + sizeof(int) * 1, sizeof(int));
				mattChunk.content.at(1) = conv.i;
				memcpy(conv.c, content.data() + sizeof(int) * 2, sizeof(float));
				mattChunk.matContent.at(0) = conv.f;
				memcpy(conv.c, content.data() + sizeof(int) * 2 + sizeof(float), sizeof(int));
				mattChunk.content.at(2) = conv.i;
				for (int m = 1; m < mattChunk.matContent.size(); ++m)
				{
					memcpy(conv.c, content.data() + sizeof(int) * 2 + sizeof(float) + sizeof(int) + sizeof(float) * (m - 1), sizeof(float));
					mattChunk.matContent.at(m) = conv.f;
				}
				appendChild(mattChunk);
				break;
			}
			default:
			{
				// currently extensions are ignored...
				break;
			}
			}
		}
		return true;
	}
	catch (int e)
	{
		return false;
	}
}
