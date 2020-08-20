"""
TODO: This.
"""
import re
import traceback
from abc import ABC
from multiprocessing import Event, Process, Queue
from queue import Empty
from threading import Thread
from time import monotonic
from traceback import print_last
from typing import Any, Callable, NamedTuple, List, Union, Type, Optional, Literal

_MESSAGE_QUEUE = Queue()
_SUBSCRIBERS = {}


class Message(NamedTuple):
    topic: str
    data: Any
    timestamp: float


class Node(ABC):
    def __init__(self, name: str, loop_period: Optional[float], worker_type: Literal['process', 'thread'] = 'thread'):
        self.name = name
        self.loop_period = loop_period

        worker_type = worker_type.casefold()
        if worker_type == 'process':
            worker_class = Process
        elif worker_type == 'thread':
            worker_class = Thread
        else:
            raise ValueError(f'worker_type must be one of "process" or "thread"; got {worker_type!r}.')

        self._stop_flag = Event()
        self._worker = worker_class(target=self.run, name=name, daemon=True)

    def __str__(self) -> str:
        return f'Node "{self.name}"'

    def start(self) -> None:
        self.log('Starting...')
        self._worker.start()

    def run(self) -> None:
        if self.loop_period is not None and self.loop_period > 0:
            while True:
                t0 = monotonic()

                try:
                    self.loop()
                except Exception:
                    self.log_error(f'Exception occurred while running loop():')
                    for line in traceback.format_exc().splitlines():
                        self.log_error(line)

                elapsed = monotonic() - t0

                timeout = self.loop_period - elapsed
                if timeout <= 0:
                    self.log_warning(
                        f'loop() took {elapsed}s to run, which is longer than the loop period of {self.loop_period}s!')

                if self._stop_flag.wait(timeout=timeout):
                    break
        else:
            while not self._stop_flag.is_set():
                self.loop()

        self.log('Stopped.')

    def loop(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        self.log('Stopping...')
        self._stop_flag.set()

    def is_alive(self) -> bool:
        return self._worker.is_alive()

    def join(self, timeout: float = None) -> None:
        self._worker.join(timeout=timeout)

    def log(self, message: str, *args, message_type: str = 'INFO', **kwargs):
        print(f'[{message_type}] {self}: {message}', *args, **kwargs)

    def log_error(self, message: str, *args, **kwargs):
        self.log(message, *args, message_type='ERROR', **kwargs)

    def log_warning(self, message: str, *args, **kwargs):
        self.log(message, *args, message_type='WARN', **kwargs)

    @staticmethod
    def publish(topic: str, data: Any) -> None:
        _MESSAGE_QUEUE.put(Message(topic, data, monotonic()))

    @staticmethod
    def subscribe(topic: str, handler: Callable[[Message], None]) -> None:
        if topic in _SUBSCRIBERS:
            handlers = _SUBSCRIBERS[topic]
            if handler not in handlers:
                handlers.append(handler)
        else:
            _SUBSCRIBERS[topic] = [handler]


class NodeRunner:
    def __init__(self, nodes: List[Node] = None):
        self.nodes = nodes if nodes else []

        self._print_message_patterns = []

    def add_node(self, node: Node) -> None:
        self.nodes.append(node)

    def print_messages_matching(self, topic_pattern: str):
        self._print_message_patterns.append(re.compile(topic_pattern))

    def run(self) -> None:
        try:
            # Start all the nodes
            for node in self.nodes:
                node.start()

            # Handle the messages pushed to the queue until the stop message is published,
            # or all nodes are no longer alive for some reason.
            while True:
                # Get the next message
                try:
                    message = _MESSAGE_QUEUE.get(block=True, timeout=2)
                except Empty:
                    if any(filter(lambda n: n.is_alive(), self.nodes)):
                        continue
                    else:
                        break

                if any(pattern.match(message.topic) for pattern in self._print_message_patterns):
                    print(message)

                if message.topic == 'easybot.stop':
                    break

                # Get the subscribed handlers, or continue if there are none
                try:
                    handlers = _SUBSCRIBERS[message.topic]
                except KeyError:
                    continue

                # Pass the message to all the handlers
                for handler in handlers:
                    try:
                        handler(message)
                    except:
                        print_last()
        finally:
            # Stop all the nodes
            stopping_nodes = []
            for node in self.nodes:
                try:
                    node.stop()
                except:
                    print_last()
                    continue

                stopping_nodes.append(node)

            for node in stopping_nodes:
                node.join(timeout=10)


def main():
    import signal

    def publish_stop_message(signal_number, stack_frame):
        Node.publish('easybot.stop', None)

    signal.signal(signal.SIGINT, publish_stop_message)
    signal.signal(signal.SIGTERM, publish_stop_message)

    class TestNode(Node):
        def __init__(self, name: str, loop_period: Optional[float],
                     worker_class: Type[Union[Thread, Process]] = Thread):
            super().__init__(name, loop_period, worker_class)

            self.subscribe('test', self.handle_test)

        def loop(self) -> None:
            self.publish('test', f'{self}: Hello!')

        def handle_test(self, message: Message) -> None:
            # self.log('I got a message!')
            pass

    class SlowNode(Node):
        def loop(self) -> None:
            import time
            time.sleep(3)
            self.publish('test', f'{self}: I am slow.')

    node_runner = NodeRunner([
        TestNode('A', 1),
        TestNode('B', 1, worker_class=Process),
        # SlowNode('Slow', 2)
    ])
    import random
    for i in range(100):
        node_runner.add_node(
            TestNode(str(i), random.random(), worker_class=Thread if random.random() > 0.25 else Process))

    node_runner.run()


if __name__ == '__main__':
    main()
