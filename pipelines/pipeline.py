from abc import ABC, abstractmethod


class Pipeline(ABC):
    graceful_shutdown: bool = False
    stopped: bool = True


    def shutdown(self):
        self.graceful_shutdown = True

    def start(self, *args, **kwargs):
        if not self.stopped:
            #cry
            pass
        try:
            self.stopped = False
            self.graceful_shutdown = False
            self._run(*args,**kwargs)
        finally:
            self.stopped = True


    @abstractmethod
    def _run(self, *args, **kwargs): ...

