{{header}}
{{#includes}}
#include <{{.}}>
{{/includes}}
{{#sources}}
#include <{{.}}>
{{/sources}}
{{precontent}}
#include <pybind11/pybind11.h>
{{postinclude}}

void {{variable.mangled_name}}(::pybind11::module& m)
{
    auto sm = m{{!
        }}{{#variable.namespace}}{{#name}}.def_submodule("{{name}}"){{/name}}{{/variable.namespace}};

    auto attr = sm{{!
        }}{{#variable.scope_without_namespace}}{{#name}}.attr("{{name}}"){{/name}}{{/variable.scope_without_namespace}};

    attr.attr("{{variable.name}}") = {{variable.qualified_name}};
}

{{postcontent}}
{{footer}}
