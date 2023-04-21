import time

try:
    import colors
except:
    import Mlogger.colors as colors
from typing import *
from dataclasses import dataclass

@dataclass
class Log:
    ttime: float
    ttype: str
    msg: str

class tracker:
    def __init__(self, filter=None) -> None:
        self.filter = filter
        self.record: List[Log] = list()

    def append(self, log: Log):
        if self.filter is None or self.filter(log) == True:
            self.record.append(log)

    def write(self, filename):
        with open(filename, 'w') as f:
            for log in self.record:
                f.write(f'{log.ttime} {log.ttype} {log.msg}\n')


class Logger:
    def __init__(self, tracker=None) -> None:
        self.tracker = tracker

    def send_to_tracker(self, msg, ttype):
        if self.tracker is None:
            return
        
        self.tracker.append(Log(time.time(), ttype, msg))

    def debug(self, msg):
        msg = str(repr(msg))
        print(''.join([
            colors.BrightBlack+time.ctime(),
            colors.Bold,
            colors.White+" [",
            colors.BrightBlue+"DEBUG",
            colors.White+"]",
            colors.Reset+" - ",
            colors.Italic+msg,
            colors.Reset
        ]))
        self.send_to_tracker(msg, "debug")
        return msg

    def info(self, msg):
        msg = str(repr(msg))
        print(''.join([
            colors.BrightBlack+time.ctime(),
            colors.Bold,
            colors.White+" [",
            colors.BrightWhite+"INFO",
            colors.White+"]",
            colors.Reset+" - ",
            colors.Italic+msg,
            colors.Reset
        ]))
        self.send_to_tracker(msg, "info")
        return msg

    def warn(self, msg):
        msg = str(repr(msg))
        print(''.join([
            colors.BrightBlack+time.ctime(),
            colors.Bold,
            colors.White+" [",
            colors.BrightYellow+"DEBUG",
            colors.White+"]",
            colors.Reset+" - ",
            colors.Italic+msg,
            colors.Reset
        ]))
        self.send_to_tracker(msg, "warn")
        return msg


    def error(self, msg):
        msg = str(repr(msg))
        print(''.join([
            colors.BrightBlack+time.ctime(),
            colors.Bold,
            colors.White+" [",
            colors.BrightRed+"ERROR",
            colors.White+"]",
            colors.Reset+" - ",
            colors.Italic+msg,
            colors.Reset
        ]))
        self.send_to_tracker(msg, "error")
        return msg

    def critical(self, msg):
        msg = str(repr(msg))
        print(''.join([
            colors.BrightBlack+time.ctime(),
            colors.Bold,
            colors.White+" [",
            colors.BrightRed+colors.Reverse+"CRITICAL",
            colors.Reset+colors.Bold+"]",
            colors.Reset+" - ",
            colors.Italic+msg,
            colors.Reset
        ]))        
        self.send_to_tracker(msg, "critical")
        return msg

    def error_handler(self, func):
        def warpper(*args, **kwds):
            try:
                result = func(*args, **kwds)
                return result
            except Exception as e:
                self.error(f"Got error # {type(e).__name__}: {str(e)}")
        return warpper
                

    
if __name__ == '__main__':
    logger = Logger()
    logger.debug("Hello world")
    logger.info("Hello world")
    logger.warn("Hello world")
    logger.error("Hello world")
    logger.critical("Hello world")

    # tracker
    tracker = tracker(filter=lambda l: l.ttype=="error")
    logger = Logger(tracker=tracker)
    logger.info("did not send to tracker")
    logger.error("send to tracker")
    print()
    logger.debug("tracker...")
    print(logger.tracker)
    for i in logger.tracker.record:
        logger.info(i)

    @logger.error_handler
    def bad_func():
        return 1/0
    
    bad_func()
    
