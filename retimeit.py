#I TAKE NO CREDIT FOR THIS CODE! all credit to the author at the link below...
import time# CREDIT: https://medium.com/pythonhive/python-decorator-to-measure-the-execution-time-of-methods-fa04cb6bb36d
def timeit(method):
    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()
        if 'log_time' in kw:
            name = kw.get('log_name', method.__name__.upper())
            kw['log_time'][name] = int((te - ts) * 1000)
        else:
            print('%r  %2.4f ms' % (method.__name__, (te - ts) * 1000))
        return result
    return timed