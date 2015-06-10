^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ddynamic_reconfigure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
