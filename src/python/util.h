#ifndef R3_PYTHON_UTIL_H_
#define R3_PYTHON_UTIL_H_
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

namespace r3 {
namespace python {
namespace util {

template <typename container_type>
struct collection_from_python
{
    typedef typename container_type::value_type content_type;

    collection_from_python()
    {
        ::boost::python::converter::registry::push_back(
            &convertible,
            &construct,
            ::boost::python::type_id<container_type>()
        );
    }
 
    static void *convertible(PyObject *object_py_raw)
    {
        PyObject *iterator_py_raw = PyObject_GetIter(object_py_raw);
        if (iterator_py_raw) {
            Py_DECREF(iterator_py_raw);
            return object_py_raw;
        } else {
            PyErr_Clear();
            return NULL;
        }
    }
 
    static void construct(
        PyObject *object_py_raw,
        boost::python::converter::rvalue_from_python_stage1_data *data)
    {
        using ::boost::python::borrowed;
        using ::boost::python::converter::rvalue_from_python_storage;
        using ::boost::python::handle;
        using ::boost::python::object;
        using ::boost::python::stl_input_iterator;
        using ::std::make_pair;

        // Wrap the PyObject in a Boost.Python class.
        object object_py(handle<>(borrowed(object_py_raw)));

        // Allocate storage for the output object using placement-new.
        void *storage
            = reinterpret_cast<rvalue_from_python_storage<container_type> *>(data)
                ->storage.bytes;

        new (storage) container_type(
            stl_input_iterator<content_type>(object_py),
            stl_input_iterator<content_type>()
        );

        data->convertible = storage;
    }
};

} // namespace util
} // namespace python
} // namespace r3

#endif
