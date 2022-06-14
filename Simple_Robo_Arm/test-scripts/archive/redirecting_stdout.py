import io
import sys

orig_stdout = sys.stdout
sys.stdout = io.StringIO()
my_stdout = sys.stdout

# examine
sys.stdout = orig_stdout
print(my_stdout)
print(my_stdout.getvalue())

# with redirect_stdout(f):
#     print('foobar')
#     print(12)
# print('Got stdout: "{0}"'.format(f.getvalue()))
