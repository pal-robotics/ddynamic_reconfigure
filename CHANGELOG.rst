^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ddynamic_reconfigure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2021-01-18)
------------------
* Merge branch 'atomic-fix' into 'erbium-devel'
  Fix missing atomic header
  See merge request control/ddynamic_reconfigure!19
* Fix missing atomic header
* Update README about c++11 requirements
* Contributors: Adria Roig, Victor Lopez, victor

0.4.0 (2021-01-12)
------------------
* Merge branch 'cosmetic_fixes' into 'erbium-devel'
  update clang format styling
  See merge request control/ddynamic_reconfigure!17
* Merge branch 'rt_thread_safe' into 'cosmetic_fixes'
  Rt thread safe
  See merge request control/ddynamic_reconfigure!18
* update assignValue method to pass every arg by const reference
* remove the deferred async from tests
* fix to update the configuration data proper way
* minor fixes to the method documentation
* update assignValue function to pass string by const reference
* Added tests for the manual update of registered variables data
* Added updateRegisteredVariablesData method to update the registered variables
* Don't update the config if manual update is chosen
* added auto_update option to the constructor
* added updateConfigData method
* changed the methods to be virtual
* update clang format styling
* Contributors: Sai Kishor Kothakota, victor

0.3.2 (2020-10-27)
------------------
* Merge branch 'pointer-with-callback' into 'erbium-devel'
  Add new function that takes pointer and a callback
  See merge request control/ddynamic_reconfigure!16
* Add new function that takes pointer and a callback
* Contributors: Victor Lopez, victor

0.3.1 (2020-10-08)
------------------
* Merge pull request #18 from clearpathrobotics/issue_17
  Reverse the order of creation of the topic publisher and rosservice
* Reverse the order of creation of the topic publisher and rosservice to prevent startup race conditions leading to crash.
  issue: #17
* Contributors: Guillaume Autran, Victor Lopez

0.3.0 (2020-05-04)
------------------
* Merge pull request #12 from eurogroep/feat/groups
  feat(groups): Option to specify a group for a variable
* feat(groups): Option to specify a group for a variable
  The group "Default" will be set as default so no behavior will change.
  However, this commit will break ABI / API compatibility.
* Contributors: Rein Appeldoorn, Victor Lopez

0.2.3 (2020-04-10)
------------------
* Merge pull request #10 from ooeygui/kinetic-devel
  Support Windows
* Windows support
* Fix for Windows
* Fix getMax(), wrong numeric_limit
* Add example for DDR with NodeHandle
* Merge pull request #7 from reinzor/patch-1
  Update README.md - show proper heading
* Update README.md
* Contributors: Lou Amadio, Rein Appeldoorn, Victor Lopez

0.2.2 (2019-09-04)
------------------
* Add SYSTEM include_directories
* Contributors: Victor Lopez

0.2.1 (2019-08-28)
------------------
* Merge branch 'fix-double-comparison' into 'erbium-devel'
  Fix double comparison
  See merge request control/ddynamic_reconfigure!14
* Fix double comparison
* Add BSD License text
* Contributors: Victor Lopez

0.2.0 (2019-03-21)
------------------
* Merge branch 'refactor-functions' into 'erbium-devel'
  Accept functions callback per individual variable
  See merge request control/ddynamic_reconfigure!12
* Extend readme
* Add string and cleanup implementation
* Add enums and callbacks
* Accept functions callback per individual variable
* Contributors: Victor Lopez

0.1.7 (2019-03-20)
------------------
* Merge branch 'fix-gmock-dependency' into 'erbium-devel'
  Rename gmock dependency for public release
  See merge request control/ddynamic_reconfigure!11
* Rename gmock dependency for public release
* Contributors: Victor Lopez

0.1.6 (2018-12-11)
------------------
* Merge branch 'ddr_debug_msg' into 'erbium-devel'
  changed info msg to debug
  See merge request control/ddynamic_reconfigure!10
* changed info msg to debug
* Change license to BSD
* Remove internal build system link
* Contributors: Hilario Tome, Victor Lopez

0.1.5 (2018-10-09)
------------------
* Merge branch 'fix-test' into 'erbium-devel'
  Fix test
  See merge request control/ddynamic_reconfigure!9
* Fix comment regarding spin_thread
* Fix failing test due to removal of spin_thread
* Contributors: Victor Lopez

0.1.4 (2018-09-17)
------------------
* Merge branch 'fix-spammy' into 'erbium-devel'
  Only publish when variables has changed
  See merge request control/ddynamic_reconfigure!8
* Only publish when variables has changed
* Contributors: Hilario Tome, Victor Lopez

0.1.3 (2018-09-14)
------------------
* Reduce update frequency to 10second
* Contributors: Victor Lopez

0.1.2 (2018-09-12)
------------------
* Merge branch 'add-auto-publish' into 'erbium-devel'
  Add auto publish of updates values to ddynamic_reconfigure
  See merge request control/ddynamic_reconfigure!7
* Add auto publish of updates values to ddynamic_reconfigure
* Contributors: Hilario Tome, Victor Lopez

0.1.1 (2018-07-26)
------------------
* Merge branch 'fix-test-crash' into 'erbium-devel'
  Fix test crash
  See merge request control/ddynamic_reconfigure!6
* Fix uninitialized variables
* Use gmock properly
* fixed merge request
* fixed merge
* Fix service response
* Contributors: Hilario Tome, Victor Lopez

0.1.0 (2018-01-15)
------------------
* formating
* made private unnecesary functions
* Merge branch 'dubnium-devel' of gitlab:control/ddynamic_reconfigure into dubnium-devel
* Formating
* Add new file
* Contributors: Hilario Tome

0.0.5 (2016-04-14)
------------------
* Merge branch 'user-callback' into 'dubnium-devel'
  User callback
  Remember that we have to re release everyone who depends on this since it breaks API.
  See merge request !1
* Add test for double param
* Add hack to have namespaced DdynamicReconfigure, for easier migration
* Add user callback and unit tests
* Migrate package to format 2
* Contributors: Hilario Tome, Victor Lopez

0.0.4 (2016-03-07)
------------------
* Added destructor, fixed bug
* Added to dynamic reconfigure to parse from param server the initial value if it is availlable
* Contributors: Hilario Tome

0.0.3 (2015-06-10)
------------------
* Added license and documentation
* Contributors: Hilario Tome

0.0.2 (2015-05-25)
------------------
* Added min and max value specification when registering a variable
* Contributors: Hilario Tome

0.0.1 (2015-01-26)
------------------
* fix author, mantainer
* move ddynamic reconfigure to standalone repo
* Prepare ddynamic_reconfigure for standalone package
* Added safe header
* Added test folder
* Fixed a bug when generating the config description, the int vector was being used in the bool part
* Added typedef for ddreconfigure
* Bug fix, now the parameters can be seen in dynamic reconfigure even if they have changed from c++
* Updated DDynamic reconfigure to published updated values persistently
* Added working momentum task
* Fixed bug, wrong return statement
* Fixed export
* Fixed bug in ddynamic reconfigure and its CmakeFile
* Minor changes to add the abstract reference to the goto dynamic tasks
* Dynamics wbc is working again (Really slowly with uquadprog) visualization of torques and partially of forces (also partial force integration)
* Added DDyanmic_reconfigure package, a way to have dynamic reconfigure functionality without a cfg
* Contributors: Hilario Tome, Luca Marchionni
