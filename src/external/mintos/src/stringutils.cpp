/*****************************************************************************
 *
 * Copyright (c) 2013, the Trustees of Indiana University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Indiana University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ***************************************************************************/
#include "stringutils.h"
#include <mintos/misc/utils.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>

char CloseBracket(char c)
{
  switch(c) {
  case '[': return ']';
  case ']': return '[';
  case '(': return ')';
  case ')': return '(';
  case '<': return '>';
  case '>': return '<';
  case '`': return '\'';
  case '\'': return '`';
  case '\\': return '/';
  case '/': return '\\';
  default: return c;
  }
}

void Lowercase(char* str)
{
	for(;*str;str++)
		(*str) = tolower(*str);
}

void Uppercase(char* str)
{
	for(;*str;str++)
		(*str) = toupper(*str);
}

void Lowercase(std::string& str)
{
	for(unsigned int i=0;i<str.length();i++)
		str[i] = tolower(str[i]);
}

void Uppercase(std::string& str)
{
	for(unsigned int i=0;i<str.length();i++)
		str[i] = toupper(str[i]);
}

std::string Strip(const std::string& str)
{
  return LStrip(RStrip(str));
}

std::string LStrip(const std::string& str)
{
  const char* chars=" \f\n\r\t\v";
  std::string::size_type index=str.find_first_not_of(chars);
  return str.substr(index);
}

std::string RStrip(const std::string& str)
{
  const char* chars=" \f\n\r\t\v";
  std::string::size_type index=str.find_last_not_of(chars);
  return str.substr(0,index+1);
}

int ReplaceAll(std::string& str,const char* strfind,const char* strreplace)
{
	size_t lenfind=strlen(strfind);
	size_t lenrep=strlen(strreplace);
	int n=0;
	size_t j=0;
	for(;(j=str.find(strfind,j))!=str.npos;n++) {
		str.replace(j,lenfind,strreplace);
		j+=lenrep;
	}
	return n;
}

bool IsValidCToken(const char* str)
{
	//RE is (_|[alpha])(_|[alnum])*
	if(!str) return false;
	if(!*str) return false;
	if(isdigit(*str))
		return false;
	while(*str) {
		if(!(isalnum(*str) || *str=='_'))
			return false;
		str++;
	}
	return true;
}

bool IsValidInteger(const char* str)
{
	//RE is [+|-]?[digit]+
	if(!str) return false;
	if(!*str) return false;
	if(*str=='-' || *str=='+') str++;
	if(!isdigit(*str)) return false;
	while(*str) {
		if(!isdigit(*str))
			return false;
		str++;
	}
	return true;
}

bool IsValidFloat(const char* str)
{
	//RE is [+|-]?([digit]+|[digit]+.[digit]*|.[digit]+)((e|E)[+|-]?[digit]+)?
	if(!str) return false;
	if(!*str) return false;
	if(*str=='-' || *str=='+') str++;
	bool readExponent = false;
	if(isdigit(*str)) {  //branch 1 [digit]+|[digit]+.[digit]*
		str++;
		bool readDot=false;
		while(*str && !readExponent) {
			if(*str == '.') {
				if(readDot) return false;
				readDot=true;
			}
			else if(*str == 'e' || *str == 'E')
				readExponent = true;
			else if(!isdigit(*str)) return false;
			str++;
		}
	}
	else if(*str == '.') { //branch 2 .[digit]+
		str++;
		if(!isdigit(*str)) return false;
		str++;
		while(*str && !readExponent) {
			if(*str == 'e' || *str == 'E')
				readExponent = true;
			else if(!isdigit(*str)) return false;
			str++;
		}
	}
	else return false;

	if(readExponent) {
		if(!IsValidInteger(str)) return false;
	}
	return true;
}

/// Detects a pattern in str = [prefix][digits][suffix]
/// Returns the integer in [digits], or -1 if no such pattern is found.
int DetectNumericalPattern(const char* str,char prefix[],char suffix[],int& numDigits)
{
  int n=strlen(str);
  int beginDigits=n,endDigits=n;
  for(int i=0;i<n;i++) {
    if(isdigit(str[i])) {
      beginDigits=i;
      break;
    }
  }
  if(beginDigits == n) return -1;
  for(int i=beginDigits;i<n;i++) {
    if(!isdigit(str[i])) {
      endDigits=i;
      break;
    }
  }
  numDigits = endDigits-beginDigits;
  strncpy(prefix,str,beginDigits);
  prefix[beginDigits] = 0;
  strncpy(suffix,str+endDigits,n-endDigits);
  suffix[n-endDigits] = 0;
  char* buf = new char[n];
  strncpy(buf,str+beginDigits,endDigits-beginDigits);
  buf[endDigits-beginDigits] = 0;
  int val = atoi(buf);
  delete buf;
  return val;
}

//For a string in the pattern XXXX####XXXX, increments the #### part.
//The number of digits can be 1-4.  For a string with no digits,
//leaves the string unchanged.
void IncrementStringDigits(char* str)
{
  char* prefix, *suffix;
  int max=strlen(str);
  prefix = new char[max+1];
  suffix = new char[max+1];
  int numDigits=0;
  int num=DetectNumericalPattern(str,prefix,suffix,numDigits);
  if(num < 0) { delete[]prefix; delete[]suffix; return; }
  if(num >= 10) numDigits = Max(numDigits,2);
  if(num >= 100) numDigits = Max(numDigits,3);
  if(num >= 1000) numDigits = Max(numDigits,4);
  if(num >= 10000) numDigits = 5;
  
  num++;
  switch(numDigits) {
  case 1:
    sprintf(str,"%s%01d%s",&prefix[0],num,&suffix[0]);
    break;
  case 2:
    sprintf(str,"%s%02d%s",&prefix[0],num,&suffix[0]);
    break;
  case 3:
    sprintf(str,"%s%03d%s",&prefix[0],num,&suffix[0]);
    break;
  case 4:
    sprintf(str,"%s%04d%s",&prefix[0],num,&suffix[0]);
    break;
  default:
    sprintf(str,"%s%d%s",&prefix[0],num,&suffix[0]);
    break;
  }
  delete[]prefix;
  delete[]suffix;
}


//For a string in the pattern XXXX####XXXX, increments the #### part.
//The number of digits can be 1-4.  For a string with no digits,
//leaves the string unchanged.
void IncrementStringDigits(std::string& str)
{
  char* prefix, *suffix;
  int max=str.length();
  prefix = new char[max+1];
  suffix = new char[max+1];
  int numDigits=0;
  int num=DetectNumericalPattern(str.c_str(),prefix,suffix,numDigits);
  if(num < 0) { delete[]prefix; delete[]suffix; return; }
  if(num >= 10) numDigits = Max(numDigits,2);
  if(num >= 100) numDigits = Max(numDigits,3);
  if(num >= 1000) numDigits = Max(numDigits,4);
  if(num >= 10000) numDigits = 5;
  
  char* buf = new char[max+1];
  num++;
  switch(numDigits) {
  case 1:
    sprintf(buf,"%s%01d%s",&prefix[0],num,&suffix[0]);
    break;
  case 2:
    sprintf(buf,"%s%02d%s",&prefix[0],num,&suffix[0]);
    break;
  case 3:
    sprintf(buf,"%s%03d%s",&prefix[0],num,&suffix[0]);
    break;
  case 4:
    sprintf(buf,"%s%04d%s",&prefix[0],num,&suffix[0]);
    break;
  default:
    sprintf(buf,"%s%d%s",&prefix[0],num,&suffix[0]);
    break;
  }
  str = buf;
  delete[]prefix;
  delete[]suffix;
  delete[]buf;
}






int LengthWithDOSEndlines(const char* str)
{
	int i=0;
	bool return_read=false;
	while(*str) {
		switch(*str) {
		case '\r':
			return_read=true;
			break;
		case '\n':
			i+=2;
			break;
		default:
			if(return_read) {
				i+=2;
				return_read=false;
			}
			i++;
			break;
		}
		str++;
	}
	if(return_read)
		i+=2;
	return i;
}

bool EndlinesToDOS(const char* str,char* out,int max)
{
	int i=0;
	bool return_read=false;
	while(*str) {
		if(i>=max) return false;
		switch(*str) {
		case '\r':
			return_read=true;
			break;
		case '\n':
			if(i+1>=max) return false;
			out[i]='\r';
			out[i+1]='\n';
			return_read=false;
			i+=2;
			break;
		default:
			if(return_read) {
				if(i+2>=max) return false;
				out[i]='\r';
				out[i+1]='\n';
				i+=2;
				return_read=false;
			}
			out[i]=*str;
			i++;
			break;
		}
		str++;
	}
	if(return_read) {
		if(i+2>=max) return false;
		out[i]='\r';
		out[i+1]='\n';
		i+=2;
		return_read=false;
	}
	if(i>=max) return false;
	out[i]=0;
	return true;
}

bool EndlinesFromDOS(const char* str,char* out,int max)
{
	int i=0;
	while(*str) {
		if(i>=max) return false;
		if(*str != '\r') out[i]=*str;
	}
	if(i>=max) return false;
	out[i]=0;
	return true;
}

void EndlinesToDOS(std::string& str)
{
	ReplaceAll(str,"\r\n","\n");  //make sure we dont duplicate \n's
	ReplaceAll(str,"\n","\r\n");
}

void EndlinesFromDOS(std::string& str)
{
	ReplaceAll(str,"\r\n","\n");
}

const char* FileExtension (const char* str)
{
	const char* dp = strrchr(str,'.');
	if(dp == NULL) return NULL;
	dp++;

	return dp;
}

void ChangeFileExtension (char* str, const char* ext)
{
	char* dp = strrchr(str,'.');
	if(dp == NULL)
	{
		strcat(str,".");
		strcat(str,ext);
		return;
	}

	dp++;

	strcpy(dp, ext);
}

const char* GetFileName(const char* str)
{
	const char* fp = strrchr(str,'\\');
	if(fp == NULL)
	{
		fp = strrchr(str,'/');
		if(fp == NULL)
			return str;
	}
	fp++;

	return fp;
}


void GetFilePath(const char* str, char* buf)
{
	strcpy(buf, str);
	char* fp = strrchr(buf,'\\');
	if(fp == NULL)
	{
		fp = strrchr(buf,'/');
		if(fp == NULL) {
		  buf[0] = 0;
		  return;
		}
	}
	fp++;
	*fp = '\0';
}

void StripExtension(char* str)
{
	char* dp = strrchr(str,'.');
	if(!dp)
		return;
	*dp = '\0';
}




std::string FileExtension (const std::string& str)
{
  size_t pos = str.rfind('.');
  if(pos == std::string::npos) return "";
  return str.substr(pos+1,str.length()-pos-1);
}

void ChangeFileExtension (std::string& str, const std::string& ext)
{
  size_t pos = str.rfind('.');
  if(pos == std::string::npos) 
    str = str + "." + ext;
  else 
    str = str.substr(0,pos+1) + ext;
}

std::string GetFileName(const std::string& str)
{
  size_t pos = str.rfind('\\');
  if(pos == std::string::npos){
    pos = str.rfind('/');
    if(pos == std::string::npos){
      return str;
    }
  }
  return str.substr(pos+1,str.length()-pos-1);
}

std::string GetFilePath(const std::string& str)
{
  size_t pos = str.rfind('\\');
  if(pos == std::string::npos){
    pos = str.rfind('/');
    if(pos == std::string::npos){
      return "";
    }
  }
  return str.substr(0,pos);
}

void StripExtension(std::string& str)
{
  size_t pos = str.rfind('.');
  if(pos == std::string::npos) 
    return;
  else 
    str = str.substr(0,pos);
}



#ifdef WIN32

void ToWideChar(const char* str, WCHAR* buf, int maxBuf)
{
	buf[0] = 0;
	int res = MultiByteToWideChar(CP_ACP, 0, str, -1, buf,maxBuf);
	if(res == 0)
	{
	  fprintf(stderr,"Couldnt' convert the string to wide characters\n");
	  abort();
	}
}

#endif //WIN32










/* 
   base64.cpp and base64.h

   Copyright (C) 2004-2008 René Nyffenegger

   This source code is provided 'as-is', without any express or implied
   warranty. In no event will the author be held liable for any damages
   arising from the use of this software.

   Permission is granted to anyone to use this software for any purpose,
   including commercial applications, and to alter it and redistribute it
   freely, subject to the following restrictions:

   1. The origin of this source code must not be misrepresented; you must not
      claim that you wrote the original source code. If you use this source code
      in a product, an acknowledgment in the product documentation would be
      appreciated but is not required.

   2. Altered source versions must be plainly marked as such, and must not be
      misrepresented as being the original source code.

   3. This notice may not be removed or altered from any source distribution.

   René Nyffenegger rene.nyffenegger@adp-gmbh.ch

*/

static const std::string base64_chars = 
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";


static inline bool is_base64(unsigned char c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

std::string base64_encode(const char* bytes_to_encode, unsigned int in_len) {
  std::string ret;
  int i = 0;
  int j = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];

  while (in_len--) {
    char_array_3[i++] = *(bytes_to_encode++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for(i = 0; (i <4) ; i++)
        ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i)
  {
    for(j = i; j < 3; j++)
      char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;

    for (j = 0; (j < i + 1); j++)
      ret += base64_chars[char_array_4[j]];

    while((i++ < 3))
      ret += '=';

  }

  return ret;

}

std::string base64_decode(const char* encoded_string, unsigned int in_len) {
  int i = 0;
  int j = 0;
  int in_ = 0;
  unsigned char char_array_4[4], char_array_3[3];
  std::string ret;

  while (in_len-- && ( encoded_string[in_] != '=') && is_base64(encoded_string[in_])) {
    char_array_4[i++] = encoded_string[in_]; in_++;
    if (i ==4) {
      for (i = 0; i <4; i++)
        char_array_4[i] = base64_chars.find(char_array_4[i]);

      char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
      char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
      char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

      for (i = 0; (i < 3); i++)
        ret += char_array_3[i];
      i = 0;
    }
  }

  if (i) {
    for (j = i; j <4; j++)
      char_array_4[j] = 0;

    for (j = 0; j <4; j++)
      char_array_4[j] = base64_chars.find(char_array_4[j]);

    char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
    char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

    for (j = 0; (j < i - 1); j++) ret += char_array_3[j];
  }

  return ret;
}

std::string ToBase64(const std::string& in)
{
  return base64_encode(in.c_str(),in.length());
}

std::string FromBase64(const std::string& in)
{
  return base64_decode(in.c_str(),in.length());
}

std::vector<std::string> Split(const std::string& str, const std::string& delim)
{
  std::vector<std::string> parts;
  size_t start, end = 0;
  while (end < str.size()) {
    start = end;
    while (start < str.size() && (delim.find(str[start]) != std::string::npos)) {
      start++;  // skip initial whitespace
    }
    end = start;
    while (end < str.size() && (delim.find(str[end]) == std::string::npos)) {
      end++; // skip to end of word
    }
    if (end-start != 0) {  // just ignore zero-length strings.
      parts.push_back(std::string(str, start, end-start));
    }
  }
  return parts;
}

void SplitPath(const std::string& path,std::vector<std::string>& elements)
{
  elements = Split(path,"\\/");
}

std::string JoinPath(const std::vector<std::string>& elements)
{
#if WIN32
  char delim = '\\';
#else
  char delim = '/';
#endif
  std::string res;
  for(size_t i=0;i<elements.size();i++) {
    if(elements[i].empty()) continue;
    size_t n=elements[i].length();
    int start = 0;
    if(elements[i][0] == '/' || elements[i][0] == '\\') 
      start = 1;
    if(elements[i][n-1] == '/' || elements[i][n-1] == '\\') 
      n--;
    res += elements[i].substr(start,n-start);
    if(i+1 < elements.size()) res += delim;
  }
  return res;
}

std::string JoinPath(const std::string& path1,const std::string& path2)
{
  std::vector<std::string> path(2);
  path[0] = path1;
  path[1] = path2;
  return JoinPath(path);
}
