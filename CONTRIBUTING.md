# Contributing to SwerveLibrary

Thanks for your interest in contributing to SwerveLibrary!

The following describes guidelines for how to contribute to this project, whether it be by filing
an issue or submitting a pull request.

## Issues

Any bugs, feature requests, etc. should be submitting to the 
[Github issues page](https://github.com/frc-88/SwerveLibrary/issues). The following are some
general rules of etiquette.

* Before posting an issue, check if someone else has already posted a similar one, and consider joining that conversation instead.
* Be as descriptive as possible. If it's a bug, provide the error message/faulty behavior and steps to reproduce. If it's a feature request, explain why we should add it.
* Use screenshots and other visual aids, when applicable.
* Be patient. Remember that this is an open source project with countless features that could be added. Just because your issue doesn't fall first on the priority list doesn't mean it's being ignored.

## Pull Requests

All mentors, students, and other members of the FRC community are welcome to submit pull requests
for consideration. They should be targeted at the develop branch.

Only a administrator of the project can merge PRs into develop or master. A PR must pass all CI
checks, which currently include building the library successfully, compiling properly formatted
javadocs, verifying compliance with the google java formatter, and building the example projects
successfully.

### Formatting

One of the requirements for all code in this repository is that it is formatted with
[the google java formatter](https://github.com/google/google-java-format). Doing so is easy;
whenver you build SwerveLibrary, it will automatically run the formatter on the entire project.
You can also run the formatter by itself with `./gradlew googleJavaFormat`.This makes it easy for
us to have consistently formatted code.

## Development environment

There are many valid ways to setup a development environment for contributing to this project. We
recommend using [git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) to add the
source code to a standard WPILIB robot project (with git) so that you can test your changes on an
actual robot. To do this, go into the root directory of your project, then run the following:

```
git submodule add https://github.com/frc-88/SwerveLibrary
git push
```

This will create a directory in your project which links to this project. Within it you can change
branches, pull/push code, and anything else you could do with a normal git repo.

Now that you have a new submodule in your git repository, all other users will have to initialize
it on their machines with:

```
git submodule update --init --recursive
```

If someone is cloning the repository new, they can run the above after cloning, or just use this:

```
git clone --recurse-submodules <repo-url>
```

From here, the process of adding SwerveLibrary to your gradle project is slightly different than
usual. You first need to add the below line to your settings.gradle file.

```groovy
include 'SwerveLibrary'
```

In build.gradle, you need to add the following line to the dependencies block.

```groovy
dependencies {
    // Other stuff

    compile project(':SwerveLibrary')
}
```

Finally, before you build your robot code, you need to build the SwerveLibrary code by entering the
SwerveLibrary directory and running the following.

```
./gradlew build
```

You will need to do this every time you make a modification to SwerveLibrary code.

## The base config

One feature that you will often need to interact with while contibuting is the base_config.toml.
This is the basis for our configuration system. Effectively, the base config gets loaded first,
then the user-specified config file adds additional fields and overrides existing ones, giving the
user complete control to customize anything in the base config. New constants that could be tuned
should preferably be added to the config.

The template system in the config is fairly self-explanatory. You can define new templates, such as
support for a type of gyro or a COTS swerve module, by adding it to the relevant templates table
and including defaults for whichever fields are desired.

Due to a difference between where Java looks for resources in simulation vs. robot
deployment, we have 2 base config files. One is at `/src/main/java/frc/team88/swerve/configuration/base_config.toml`,
while the other is at `src/main/resources/frc/team88/swerve/configuration/base_config.toml`. At
this point, these 2 files need to manually be kept the same.

More information on the TOML format, which we use for configs, can be found
[here](https://toml.io/en/).
