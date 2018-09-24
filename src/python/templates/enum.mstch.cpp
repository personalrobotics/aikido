{{{header}}}
{{#includes}}
#include <{{{.}}}>
{{/includes}}
{{#sources}}
#include <{{.}}>
{{/sources}}
{{{precontent}}}
#include <pybind11/pybind11.h>
{{postinclude}}

void {{enum.mangled_name}}(pybind11::module& m)
{
    auto sm = m{{!
        }}{{#enum.namespace}}{{#name}}.def_submodule("{{name}}"){{/name}}{{/enum.namespace}};

    auto attr = sm{{!
        }}{{#enum.scope_without_namespace}}{{#name}}.attr("{{name}}"){{/name}}{{/enum.scope_without_namespace}};

    ::pybind11::enum_<{{{enum.type}}}>(attr, "{{enum.name}}"){{#enum.values}}
        .value("{{name}}", {{{qualified_name}}}){{/enum.values}}
        .export_values();
}
{{{postcontent}}}
{{{footer}}}
