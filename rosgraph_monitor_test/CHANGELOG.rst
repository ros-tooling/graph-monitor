^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosgraph_monitor_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-10-07)
------------------
* feat: subscribe /parameter_events to get updated parameter info (`#72 <https://github.com/ros-tooling/graph-monitor/issues/72>`_)
* feat: fetch initial parameter values alongside descriptors (`#71 <https://github.com/ros-tooling/graph-monitor/issues/71>`_)
* fix: flaky launch tests with more robust message collector utility (`#67 <https://github.com/ros-tooling/graph-monitor/issues/67>`_)
* feat: fetch parameter descriptors after names arrive (`#66 <https://github.com/ros-tooling/graph-monitor/issues/66>`_)
* fix: lyrical+rolling tests (`#57 <https://github.com/ros-tooling/graph-monitor/issues/57>`_)
* feat!: migrate to new rosgraph_msgs for reporting graph structure (`#54 <https://github.com/ros-tooling/graph-monitor/issues/54>`_)
* lint: Move from ament linters to polymath_code_standard autoformatters (`#53 <https://github.com/ros-tooling/graph-monitor/issues/53>`_)
* Contributors: Emerson Knapp

0.2.3 (2025-10-22)
------------------
* Fix circular dependency (`#38 <https://github.com/ros-tooling/graph-monitor/issues/38>`_)
* Contributors: Emerson Knapp

0.2.2 (2025-10-16)
------------------
* Fix release builds (`#36 <https://github.com/ros-tooling/graph-monitor/issues/36>`_)
  * make tests pass against FastRTPS with its missing History QoS
* Contributors: Emerson Knapp

0.2.1 (2025-10-15)
------------------

0.2.0 (2025-08-07)
------------------
* Add coverage query node parameters (`#26 <https://github.com/ros-tooling/graph-monitor/issues/26>`_)
* Migrate launch testing to dedicated space and add coverage to the newly generated `/rosgraph` topic (`#23 <https://github.com/ros-tooling/graph-monitor/issues/23>`_)
* Contributors: Troy Gibb, Emerson Knapp
