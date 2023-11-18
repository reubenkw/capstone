from functools import wraps
from time import time


def timing(print_args=True):
    def decorator(f):
        @wraps(f)
        def wrap(*args, **kw):
            ts = time()
            result = f(*args, **kw)
            te = time()
            if print_args:
                print(
                    "func:%r args:[%r, %r] took: %2.4f sec"
                    % (f.__name__, args, kw, te - ts)
                )
            else:
                print("func:%r took: %2.4f sec" % (f.__name__, te - ts))
            return result

        return wrap

    return decorator
