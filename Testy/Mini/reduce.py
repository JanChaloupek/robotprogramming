import ast
import sys

class NameShortener(ast.NodeTransformer):
    def __init__(self, reserved_names):
        self.reserved_names = reserved_names
        self.name_map = {}
        self.counter = 0

    def get_short_name(self, original_name):
        if original_name in self.reserved_names:
            return original_name
        if original_name not in self.name_map:
            self.name_map[original_name] = f"v{self.counter}"
            self.counter += 1
        return self.name_map[original_name]

    def visit_FunctionDef(self, node):
        node.name = self.get_short_name(node.name)
        node.args.args = [self.visit(arg) for arg in node.args.args]
        self.generic_visit(node)
        return node

    def visit_ClassDef(self, node):
        node.name = self.get_short_name(node.name)
        self.generic_visit(node)
        return node

    def visit_Name(self, node):
        if isinstance(node.ctx, ast.Load) and node.id in self.reserved_names:
            return node
        node.id = self.get_short_name(node.id)
        return node

    def visit_arg(self, node):
        node.arg = self.get_short_name(node.arg)
        return node

def main(input_file, output_file):
    with open(input_file, 'r') as file:
        code = file.read()

    reserved_names = set()
    tree = ast.parse(code)
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                reserved_names.add(alias.name)
        elif isinstance(node, ast.ImportFrom):
            for alias in node.names:
                reserved_names.add(alias.name)

    shortener = NameShortener(reserved_names)
    new_tree = shortener.visit(tree)
    new_code = ast.unparse(new_tree)

    with open(output_file, 'w') as file:
        file.write(new_code)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python script.py <input_file> <output_file>")
    else:
        main(sys.argv[1], sys.argv[2])
