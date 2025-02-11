import itertools
import re
from pathlib import Path

from parsimonious.grammar import Grammar
from parsimonious.nodes import NodeVisitor

grammar = Grammar(r"""
cib_component_file = (cib_config_statement / line_ignored)+ ws
cib_config_statement = cib_config_opening_bracket cib_extend_func line_ignored
line_ignored = ~"[^\n]*\n"

cib_extend_func =  opening_bracket pipeline_list closing_bracket
pipeline_list = pipeline (delimiter pipeline)*
pipeline = action (pipe action)+
action = namespace* action_name
action_name = snake_case
namespace = snake_case "::"

cib_config_opening_bracket = ws ("static constexpr" / "constexpr static") " auto config" ws "=" ws "cib::config(" ws
opening_bracket ="cib::extend<RuntimeInit>(" ws
closing_bracket = ws ")"

delimiter = "," ws
pipe = ws ">>" ws
snake_case = ~"[a-zA-Z_][a-zA-Z0-9_]+"
ws = ~r"[\s\n]*"
""")

Pipeline = list[str]


class FlowExtractor(NodeVisitor):
    def visit_cib_component_file(self, _, visited_children) -> list[Pipeline]:
        pipeline_list: list[Pipeline] = []
        for (stmt,) in visited_children[0]:
            if stmt is not None:
                pipeline_list.extend(stmt)

        return pipeline_list

    def visit_cib_config_statement(self, _, visited_children) -> list[Pipeline]:
        return visited_children[1]

    def visit_cib_extend_func(self, _, visited_children) -> list[Pipeline]:
        first_pipeline, other_pipelines = visited_children[1]
        pipeline_list: list[Pipeline] = [first_pipeline]
        for _, pipeline in other_pipelines:
            pipeline_list.append(pipeline)

        return pipeline_list

    def visit_pipeline(self, _, visited_children) -> Pipeline:
        first_action, other_items = visited_children
        pipeline: Pipeline = [first_action]
        for _, action in other_items:
            pipeline.append(action)

        return pipeline

    def visit_line_ignored(self, _, __) -> None:
        pass

    def visit_action(self, _, visited_children) -> str:
        return visited_children[-1]

    def visit_snake_case(self, node, _) -> str:
        return node.text

    def generic_visit(self, node, visited_children):
        return visited_children or node


flow_extractor = FlowExtractor()


def extractPipelines(source_code: str) -> list[Pipeline]:
    # Eliminate all inline comments.
    source_code = re.sub(r"//[^\n]*$", "", source_code, flags=re.MULTILINE)

    # Parse the C++ file into abstract syntax tree
    ast = grammar.parse(source_code + "\n")

    # Extract the action pipeline only
    return flow_extractor.visit(ast)


def generateDigraph(pipeline_list: list[Pipeline]) -> None:
    print('digraph "RuntimeInit" {')

    # Extract nodes
    for action in set(itertools.chain(*pipeline_list)):
        print(f'    {action} [label="{action}"]')

    # Extact edges
    for pipeline in pipeline_list:
        if len(pipeline) < 2:
            print(f"// Skipping edge: {pipeline}")
            continue

        print("    " + " -> ".join(pipeline))

    print("}")


if __name__ == "__main__":
    pipeline_list: list[Pipeline] = []
    for hpp_filename in Path("../components").glob("*.hpp"):
        # print(hpp_filename.absolute().as_posix())
        with open(hpp_filename, "r") as f:
            source_code = f.read()

        pipeline_list.extend(extractPipelines(source_code))

    generateDigraph(pipeline_list)
