// PX2Base64.hpp

#ifndef PX2BASE64_HPP
#define PX2BASE64_HPP

#include "PX2UnityPre.hpp"

namespace PX2
{

	class PX2_ENGINE_ITEM Base64
	{
	public:
		Base64();
		~Base64();

		// bufLength =0时，使用d的size()
		static std::string Encode(const std::string& d, bool base64url = false, unsigned int bufLength = 0);
		static std::string Decode(const std::string base64, bool base64url = false, unsigned int bufLength = 0);
	};

}

#endif /* BASE64_H_C0CE2A47_D10E_42C9_A27C_C883944E704A */