# 2026 Robot

## Javadoc

Javadoc is auto-generated from the main branch and is available [here](https://roboblazers7617.github.io/2026Robot/index.html).

## Code Standards

Code Standards are available [here](https://roboblazers7617.github.io/code-standards/).

Contributions are specified in the code standards [here](https://roboblazers7617.github.io/code-standards/docs/git-workflow.html).

## Issues

Can be tracked on the [issues tab](https://github.com/roboblazers7617/2026Robot/issues) and in the [projects panel](https://github.com/orgs/roboblazers7617/projects/11).

## Formatting

This repository uses [spotless](https://github.com/diffplug/spotless/tree/main/plugin-gradle) to enforce the [formatting file](https://github.com/roboblazers7617/2026Robot/blob/turret/eclipse-formatter.xml), as well as a few other requirements. If you're getting a format violation error, resolving it is as simple as just running `./gradlew spotlessApply` in your terminal! That will automatically fix any format violations that spotless found. Do make sure you save everything first though, if you don't you can potentially lose stuff (the whole process would be save all, build, then spotlessApply).
