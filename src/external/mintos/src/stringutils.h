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
#ifndef UTILS_STRING_H
#define UTILS_STRING_H

#include <string>
#include <vector>

/** @file utils/stringutils.h
 * @ingroup Utils
 * @brief Utilities for string manipulation.
 */

/** @addtogroup Utils */
/*@{*/

///Returns a "close bracket" character opposite c
char CloseBracket(char c);

///Turns the string into lower/uppercase
void Lowercase(char* str);
void Uppercase(char* str);
void Lowercase(std::string& str);
void Uppercase(std::string& str);

//strips whitespace from str
std::string Strip(const std::string& str);
std::string LStrip(const std::string& str);
std::string RStrip(const std::string& str);

//divides a string into multiple strings given a string containing deliminators
std::vector<std::string> Split(const std::string& str, const std::string& delim);

///Replace all instances of strfind with strreplace in str
int ReplaceAll(std::string& str,const char* strfind,const char* strreplace);

bool IsValidCToken(const char* str);
bool IsValidInteger(const char* str);
bool IsValidFloat(const char* str);

///Detects a pattern in str = [prefix][digits][suffix].
///Returns the number specified by [digits], or -1 if no such pattern is found.
int DetectNumericalPattern(const char* str,char prefix[],char suffix[],int& numDigits);

//For a string in the pattern XXXX####XXXX, increments the #### part.
//The number of digits can be 1-4.  For a string with no digits,
//leaves the string unchanged.
void IncrementStringDigits(char* str);
void IncrementStringDigits(std::string& str);

std::string ToBase64(const std::string& in);
std::string FromBase64(const std::string& in);

///Dos-unix endline conversion
int LengthWithDOSEndlines(const char* str);
bool EndlinesToDOS(const char* str,char* out,int max);
bool EndlinesFromDOS(const char* str,char* out,int max);
void EndlinesToDOS(std::string& str);
void EndlinesFromDOS(std::string& str);

///Returns pointer to "ext" for str="filename.ext"
const char* FileExtension (const char* str);
///Replaces the file extension of str with ext, or concatenates .ext onto str
void ChangeFileExtension (char* str, const char* ext);
///Returns "file.ext" for the str="dir1/dir2/.../file.ext"
const char* GetFileName(const char* str);
///Extracts the path from str (formatted as above) into buf
void GetFilePath(const char* str, char* buf);
///Removes the file extension of str
void StripExtension(char* str);

///Returns "ext" for str="filename.ext"
std::string FileExtension (const std::string& str);
///Replaces the file extension of str with ext, or concatenates .ext onto str
void ChangeFileExtension (std::string& str, const char* ext);
///Returns "file.ext" for the str="dir1/dir2/.../file.ext"
std::string GetFileName(const std::string& str);
///Extracts the path from str (formatted as above) into buf
std::string GetFilePath(const std::string& str);
///Removes the file extension of str
void StripExtension(std::string& str);

///For a path dir1/dir2/.../file, splits the path into dir1, dir2, ..., file
///in a cross platform manner
void SplitPath(const std::string& path,std::vector<std::string>& elements);
///For elements dir1, dir2, ..., file, returns dir1/dir2/.../file
///in a cross-platform manner.  The elements may also have leading/trailing
///path delimiters which are ignored
std::string JoinPath(const std::vector<std::string>& path);
std::string JoinPath(const std::string& path1,const std::string& path2);

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
void ToWideChar(const char* str, WCHAR* buf, int maxBuf);
#endif //WIN32

/*@}*/

#endif
