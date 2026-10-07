^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosgraph_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-10-07)
------------------
* feat: subscribe /parameter_events to get updated parameter info (`#72 <https://github.com/ros-tooling/graph-monitor/issues/72>`_)
* Rebuild graph state after dynamic configuration changes (`#73 <https://github.com/ros-tooling/graph-monitor/issues/73>`_)
* feat: fetch initial parameter values alongside descriptors (`#71 <https://github.com/ros-tooling/graph-monitor/issues/71>`_)
* fix: error handling in parameter client callbacks (`#68 <https://github.com/ros-tooling/graph-monitor/issues/68>`_)
* fix: retry first parameter queries faster, with backoff (`#69 <https://github.com/ros-tooling/graph-monitor/issues/69>`_)
* feat: fetch parameter descriptors after names arrive (`#66 <https://github.com/ros-tooling/graph-monitor/issues/66>`_)
* refactor: pull out parameter collection into a dedicated component (`#61 <https://github.com/ros-tooling/graph-monitor/issues/61>`_)
* fix: Event and MutexProtected minor issues (`#65 <https://github.com/ros-tooling/graph-monitor/issues/65>`_)
* refactor: ParameterServiceClient interface instead of async query_params_func (`#64 <https://github.com/ros-tooling/graph-monitor/issues/64>`_)
* feat: track whole node repr and departed nodes in update (`#63 <https://github.com/ros-tooling/graph-monitor/issues/63>`_)
* ci: add tsan build - fix race condition issues (`#58 <https://github.com/ros-tooling/graph-monitor/issues/58>`_)
* fix: lyrical+rolling tests (`#57 <https://github.com/ros-tooling/graph-monitor/issues/57>`_)
* feat!: migrate to new rosgraph_msgs for reporting graph structure (`#54 <https://github.com/ros-tooling/graph-monitor/issues/54>`_)
* lint: Move from ament linters to polymath_code_standard autoformatters (`#53 <https://github.com/ros-tooling/graph-monitor/issues/53>`_)
* Include <cstring> (`#51 <https://github.com/ros-tooling/graph-monitor/issues/51>`_)
* Add new methods added to graph interface in Rolling (`#48 <https://github.com/ros-tooling/graph-monitor/issues/48>`_)
* Add documentation skeleton for all released packages (`#44 <https://github.com/ros-tooling/graph-monitor/issues/44>`_)
* Update READMEs (`#43 <https://github.com/ros-tooling/graph-monitor/issues/43>`_)
* Use ros_environment to detect distro instead of ament_cmake version (`#39 <https://github.com/ros-tooling/graph-monitor/issues/39>`_)
* Contributors: Emerson Knapp, Michal Sojka, Parth Oza, shrujan

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
