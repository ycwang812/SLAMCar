// PX2Base64.cpp

#include "PX2Base64.hpp"
using namespace PX2;

//----------------------------------------------------------------------------
static const std::string base64_chars =
"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
"abcdefghijklmnopqrstuvwxyz"
"0123456789+/";
static inline bool is_base64(const char c)
{
	return (isalnum(c) || (c == '+') || (c == '/'));
}
//----------------------------------------------------------------------------
Base64::Base64()
{
}
//----------------------------------------------------------------------------
Base64::~Base64()
{
}
//----------------------------------------------------------------------------
std::string Base64::Encode(const std::string& d, bool base64url, 
	unsigned int bufLength)
{
	const char alphabet_base64[] = "ABCDEFGH" "IJKLMNOP" "QRSTUVWX" "YZabcdef"
		"ghijklmn" "opqrstuv" "wxyz0123" "456789+/";
	const char alphabet_base64url[] = "ABCDEFGH" "IJKLMNOP" "QRSTUVWX" "YZabcdef"
		"ghijklmn" "opqrstuv" "wxyz0123" "456789-_";
	const char *const alphabet = base64url ? alphabet_base64url : alphabet_base64;
	const char padchar = '=';
	int padlen = 0;

	if (0 == bufLength)
		bufLength = d.size();

	std::string tmp;
	tmp.resize((bufLength + 2) / 3 * 4);

	unsigned int i = 0;
	char *out = &tmp[0];
	while (i < bufLength) {
		// encode 3 bytes at a time
		int chunk = 0;
		chunk |= ((unsigned char)d[i++]) << 16;
		if (i == bufLength) {
			padlen = 2;
		}
		else {
			chunk |= ((unsigned char)d[i++]) << 8;
			if (i == bufLength)
				padlen = 1;
			else
				chunk |= (unsigned char)d[i++];
		}

		int j = (chunk & 0x00fc0000) >> 18;
		int k = (chunk & 0x0003f000) >> 12;
		int l = (chunk & 0x00000fc0) >> 6;
		int m = (chunk & 0x0000003f);
		*out++ = alphabet[j];
		*out++ = alphabet[k];

		if (padlen > 1) {
			//if ((options & OmitTrailingEquals) == 0)
			*out++ = padchar;
		}
		else {
			*out++ = alphabet[l];
		}
		if (padlen > 0) {
			//if ((options & OmitTrailingEquals) == 0)
			*out++ = padchar;
		}
		else {
			*out++ = alphabet[m];
		}
	}
	//assert(/*(options & OmitTrailingEquals) ||*/ (out == tmp.size() + tmp.data()));
	//if (options & OmitTrailingEquals)
	//    tmp.truncate(out - tmp.data());
	return tmp;
}
//----------------------------------------------------------------------------
std::string Base64::Decode(const std::string base64, bool base64url,
	unsigned int bufLength)
{
	unsigned int buf = 0;
	int nbits = 0;
	std::string tmp;
	if (0 == bufLength)
		bufLength = base64.size();

	tmp.resize((bufLength * 3) / 4);


	unsigned int offset = 0;
	for (unsigned int i = 0; i < bufLength; ++i) {
		int ch = base64.at(i);
		int d;

		if (ch >= 'A' && ch <= 'Z')
			d = ch - 'A';
		else if (ch >= 'a' && ch <= 'z')
			d = ch - 'a' + 26;
		else if (ch >= '0' && ch <= '9')
			d = ch - '0' + 52;
		else if (ch == '+' && (base64url) == 0)
			d = 62;
		else if (ch == '-' && (base64url) != 0)
			d = 62;
		else if (ch == '/' && (base64url) == 0)
			d = 63;
		else if (ch == '_' && (base64url) != 0)
			d = 63;
		else
			d = -1;

		if (d != -1) {
			buf = (buf << 6) | d;
			nbits += 6;
			if (nbits >= 8) {
				nbits -= 8;
				tmp[offset++] = buf >> nbits;
				buf &= (1 << nbits) - 1;
			}
		}
	}

	//tmp.truncate(offset);
	if (offset < tmp.size())
		tmp.resize(offset);

	return tmp;
}
//----------------------------------------------------------------------------