# Copyright 2024 Bonsai Robotics, Inc - All Rights Reserved
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from abc import ABC, abstractmethod
from typing import List


class Measurement:
    """A single metric."""

    def __init__(self, name, tags: dict, fields: dict):
        self.name = name
        self.tags = tags
        self.fields = fields

    def __repr__(self):
        return f'Measuremen: {self.name}, {self.tags}, {self.fields}'


class MetricCollector(ABC):
    """Abstract base class defining metric collector API."""

    @abstractmethod
    def get_measurements(self) -> List[Measurement]:
        """Return a list of any new pending measurement objects."""
        pass

    @abstractmethod
    def stop(self) -> None:
        """Clean up any resources that need manual clean up."""
        pass
