# R3 Style Guide #

The code in this library should generally follow the rules of the Google C++ and Python Style Guides:

* [Google C++ Style Guide](https://google-styleguide.googlecode.com/svn/trunk/cppguide.html)
* [Google Python Style Guide](https://google-styleguide.googlecode.com/svn/trunk/pyguide.html)

There are a few notable deviations from these guides, which are detailed below.

## C++ Style ##

### Streams ###
Use streams.  They are OK.

### Exceptions ###
Use exceptions.  DART does not use exceptions though, so expect that this would need to be changed for any code that is moved upstream to DART.

### Boost ###
While Boost is a great set of libraries, we would like R3 to be portable with minimal external dependencies.  As such, please only use supported C++11 features and the following Boost libraries:

* `Boost.Python`
