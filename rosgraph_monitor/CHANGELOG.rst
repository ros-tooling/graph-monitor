^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosgraph_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.3 (2025-10-22)
------------------

0.2.2 (2025-10-16)
------------------
* Fix release builds (`#36 <https://github.com/ros-tooling/graph-monitor/issues/36>`_)
  * don't depend on ROS_DISTRO environment variable, instead use detected package versions
* Contributors: Emerson Knapp

0.2.1 (2025-10-15)
------------------
* Fix dependency spec in package xmls, for buildfarm fixing (`#35 <https://github.com/ros-tooling/graph-monitor/issues/35>`_)
* Contributors: Emerson Knapp

0.2.0 (2025-08-07)
------------------
* Node Parameters on `/rosgraph` (`#26 <https://github.com/ros-tooling/graph-monitor/issues/26>`_)
  * Graph monitor asynchronously queries parameter list for each tracked node, to know parameter names in graph representation
* Implement publisher/subscriber attributes on `/rosgraph` (`#23 <https://github.com/ros-tooling/graph-monitor/issues/23>`_)
  * Includes QoS profile mapping and enums
* Publish node info under `/rosgraph` (`#20 <https://github.com/ros-tooling/graph-monitor/issues/20>`_)
* Update package maintainers  (`#19 <https://github.com/ros-tooling/graph-monitor/issues/19>`_)
  * Update package maintainers and add some author attributions to package.xml
* Add useful set of precommit formatters and checks that enhance the ament linting (`#21 <https://github.com/ros-tooling/graph-monitor/issues/21>`_)
* Use DiagnosticAggregator instead of custom Analyzer (`#16 <https://github.com/ros-tooling/graph-monitor/issues/16>`_)
* Test the rmw_stats_shim topic_statistics in the CI build (`#18 <https://github.com/ros-tooling/graph-monitor/issues/18>`_)
* Update package maintainers  (`#19 <https://github.com/ros-tooling/graph-monitor/issues/19>`_)
  * Update package maintainers and add some author attributions to package.xml
* Use new ParamListener usercallback to skip the additional OnSetParametersCallback logic (`#13 <https://github.com/ros-tooling/graph-monitor/issues/13>`_)
* Contributors: Emerson Knapp, Troy Gibb

0.1.2 (2025-05-12)
------------------
* Kilted support (`#6 <https://github.com/ros-tooling/graph-monitor/issues/6>`_)
* Contributors: Emerson Knapp

0.1.1 (2025-04-09)
------------------
* Remove telegraf bridge and update some language
* RAII initialization of RosgraphMonitor (`#12 <https://github.com/ros-tooling/graph-monitor/issues/12>`_)
* Fix build issues with latest generate_parameter_library (`#11 <https://github.com/ros-tooling/graph-monitor/issues/11>`_)
* Action CI - support Humble, Jazzy, Rolling (`#1 <https://github.com/ros-tooling/graph-monitor/issues/1>`_)
* Initial package setup
* Contributors: Emerson Knapp, Troy Gibb, Joshua Whitley
