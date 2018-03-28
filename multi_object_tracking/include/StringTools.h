#ifndef STRINGTOOLS_H_INCLUDED
#define STRINGTOOLS_H_INCLUDED

#include <string>
#include <sstream>
#include <cstdio>
#include <vector>

template<typename T>
std::string to_string ( const T & t )
{
   std::ostringstream ss;
   ss << t;
   return ss.str();
}

template<typename T>
std::string to_string_fmt ( const char * fmt, const T & t )
{
   char buffer[1000];
   
   sprintf ( buffer, fmt, t );
   
   return std::string ( buffer );
}

#include <stdarg.h>  // For va_start, etc.

static std::string string_printf ( const std::string fmt, ... )
{
   int size = ( ( int ) fmt.size() ) * 2 + 50; // Use a rubric appropriate for your code
   std::string str;
   va_list ap;
   
   while ( 1 )     // Maximum two passes on a POSIX system...
   {
      str.resize ( size );
      va_start ( ap, fmt );
      int n = vsnprintf ( ( char * ) str.data(), size, fmt.c_str(), ap );
      va_end ( ap );
      
      if ( n > -1 && n < size )  // Everything worked
      {
         str.resize ( n );
         return str;
      }
      
      if ( n > -1 ) // Needed size returned
         size = n + 1;   // For null char
         else
            size *= 2;      // Guess at a larger size (OS specific)
   }
   
   return str;
}

template<typename T>
void from_string ( const std::string & str, T * out )
{
   std::istringstream ss ( str );
   
   ss >> *out;
}

template<typename T>
T from_string ( const std::string & str )
{
   T tmp;
   from_string ( str, &tmp );
   return tmp;
}


template<typename T>
std::string to_string ( const std::vector<T> & t )
{
   std::ostringstream ss;
   
   for ( size_t idx = 0; idx < t.size(); ++idx )
   {
      ss << t[idx];
      
      if ( idx < t.size() - 1 )
         ss << " ";
      else
         ss << std::endl;
   }
   
   return ss.str();
}

std::vector<std::string> split_string ( const std::string & str,
                                        const std::string & delimiter );

/**
 * Returns everything that comes after the last occurance of delimiter in the string. Returns
 * the entire string, if delimiter does not occur.
 */
std::string substring_after_last ( const std::string & str, const std::string & delimiter );
std::string substring_before_last ( const std::string & str, const std::string & delimiter );

/**
 * Returns everything that comes before the first occurance of delimiter in the string. Returns
 * the entire string, if delimiter does not occur.
 */
std::string substring_before_first ( const std::string & str, const std::string & delimiter );
std::string substring_after_last ( const std::string & str, const std::string & delimiter );

// trim from start
std::string ltrim ( const std::string & s );

// trim from end
std::string rtrim ( const std::string & s );

// trim from both ends
std::string trim ( const std::string & s );

#endif // STRINGTOOLS_H_INCLUDED
