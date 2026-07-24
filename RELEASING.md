# Releasing rtest

This repository has two independent release mechanisms. They are easy to confuse, and mixing them up is how a version
can appear "released" while `apt install ros-<distro>-rtest` still installs the previous one.

1. **GitHub Release with `.deb` artifacts.** The workflow [`.github/workflows/ros2-release.yml`](.github/workflows/ros2-release.yml)
is started by hand from the Actions tab (`workflow_dispatch`). It builds `.deb` packages from source for humble,
jazzy, kilted, and lyrical, and attaches them to a GitHub Release. It does not update the official ROS package index,
so it has no effect on what `apt` installs from `packages.ros.org`.

2. **The official ROS distro release**, using [bloom](http://wiki.ros.org/bloom) and a pull request against
[`ros/rosdistro`](https://github.com/ros/rosdistro). This is what publishes new versions to the ROS build farm and to
`apt install ros-<distro>-rtest`. The `pixi run -e release ...` tasks in this document drive this path.

The rest of this document is about (2). GitHub `.deb` artifacts are optional and described at the end.

## Does every new version need a manual bloom-release? Is it per-distro?

Yes to both. Nothing watches this repository's tags and runs bloom for you.

`rtest` already has a release repository (`ros2-gbp/rtest-release`) and one bloom track per ROS distro (`rolling`,
`jazzy`, `kilted`, `lyrical`; see that repo's `tracks.yaml`). Registering a track is a one-time step per distro and is
already done.

Bumping `package.xml` and pushing a git tag does **not**, by itself, update the build farm. For every new version you
must run `bloom-release` once for each ROS distro you want the version published on. Each run exports the matching git
tag, regenerates the packaging branches, and opens a new pull request against `ros/rosdistro`. Bloom has no "release
this version everywhere" mode.

Rosdistro pull requests for a package that has been released before are usually merged by CI after the sanity checks
pass. You still have to open one pull request per distro per version.

At any time you can run `pixi run -e release release-status` to see which distros are behind the version in the local
`package.xml`.

## One-time machine setup

No Docker is required. Everything runs in a dedicated pixi environment:

```bash
pixi install -e release
pixi run -e release rosdep-setup   # needs sudo the first time only, then it is idempotent
```

You will also need, once:

- SSH push access to [`ros2-gbp/rtest-release`](https://github.com/ros2-gbp/rtest-release). Ask an existing maintainer
to add you as a collaborator. Bloom pushes the generated packaging branches there.
- A GitHub personal access token for opening the `ros/rosdistro` pull request, with at least the `public_repo` scope.
Add `workflow` as well if you already have a fork of `ros/rosdistro` and want bloom to update it. `bloom-release`
prompts for the token the first time and caches it under `~/.config/bloom`.

To check that the environment is set up correctly:

```bash
pixi run -e release bloom-release rolling -- --pretend --non-interactive
```

If that command finishes cleanly, setup is fine. The `--pretend` flag means nothing is pushed.

## Releasing a new version

Follow these steps in order. Bloom does not write `CHANGELOG.rst`, bump `package.xml`, or create git tags. It only
consumes a git tag that already exists on the remote.

Only the `rtest` package is bloom-released, but `rtest`, `rtest_examples_interfaces`, and `examples` must stay on the
same version. The `prepare-release` task bumps all three `package.xml` files together.

Do not edit `<version>` in `package.xml` by hand, and do not rename the `Forthcoming` heading in `CHANGELOG.rst`.
If you have already bumped the versions yourself, skip to [If you already bumped package.xml by hand](#if-you-already-bumped-packagexml-by-hand)
instead of steps 2–5.

### 1. Start from a clean checkout of the tracked branch

Release from `main` (or whichever branch the bloom tracks follow), not from a feature branch.

```bash
git checkout main
git pull
git status
pixi run -e release release-status
```

`git status` must report a clean working tree. If `package.xml` is already newer than the latest git tag because you
edited it by hand, use the alternative section linked above.

### 2. Generate changelog entries from git history

```bash
pixi run -e release generate-changelog
```

This prepends a `Forthcoming` section, filled from commits since the last tag, to:

- `rtest/CHANGELOG.rst`
- `examples/CHANGELOG.rst`
- `rtest_examples_interfaces/CHANGELOG.rst`

### 3. Edit the new entries, then commit the changelogs

Open those three files and clean up the new `Forthcoming` block. Leave the heading as exactly `Forthcoming`;
`prepare-release` looks for that word and replaces it with the version and date. Remove noise such as `Co-authored-by`
lines, merge commits, and "generate changelogs". Keep the changes that users should know about.

Commit only the changelog files. The working tree must be clean before the next step.

```bash
git add rtest/CHANGELOG.rst examples/CHANGELOG.rst rtest_examples_interfaces/CHANGELOG.rst
git commit -m "generate changelogs"
```

### 4. Bump the version, retitle Forthcoming, and create a local tag

Choose one bump level:

```bash
pixi run -e release prepare-release          # patch, e.g. 0.2.4 -> 0.2.5
pixi run -e release prepare-release minor    # 0.2.4 -> 0.3.0
pixi run -e release prepare-release major    # 0.2.4 -> 1.0.0
```

This does not push. Before continuing, confirm:

```bash
grep -n '<version>' rtest/package.xml examples/package.xml rtest_examples_interfaces/package.xml
```

All three versions should be identical and newer than before.

```bash
head -20 rtest/CHANGELOG.rst
```

The first section heading should now be `X.Y.Z (YYYY-MM-DD)`, not `Forthcoming`.

```bash
git tag --list '0.*' | tail
```

A tag should exist whose name is exactly that version (for example `0.2.5`).

If something is wrong, delete the local tag (`git tag -d <version>`), reset the prepare-release commit (`git reset --hard HEAD~1`),
and run step 4 again. If the changelog commit is also wrong, reset that one too and go back to step 2.

### 5. Push the version commit and the tag

Bloom reads the tag from GitHub, not from your local repository.

```bash
git push
git push origin <version>    # for example: git push origin 0.2.5
```

Confirm that the tag is visible on GitHub before running bloom.

### 6. Run bloom-release for every distro

A version is not on the build farm until there is a rosdistro pull request for each distro. Pushing a git tag is not
sufficient. Run all four of the following commands and do not skip any of them.

Each run is interactive. Bloom will show diffs and ask for confirmation before pushing branches and tags to
`ros2-gbp/rtest-release`, and again before opening the `ros/rosdistro` pull request.

```bash
pixi run -e release bloom-release rolling
pixi run -e release bloom-release jazzy
pixi run -e release bloom-release kilted
pixi run -e release bloom-release lyrical
```

When bloom prompts `GitHub username [<something>]:`, type your GitHub login. The default is taken from your
operating-system username and is often wrong.

`pixi run -e release bloom-release-all` runs the same four releases with a yes/no prompt per distro. If you use it,
answer yes for every distro.

A dry run for a single distro (nothing is pushed):

```bash
pixi run -e release bloom-release rolling -- --pretend
```

### 7. Verify

```bash
pixi run -e release release-status
```

Pull requests for a package that has already been released are typically merged by CI. After they merge, the build farm
still needs time to publish the Debian packages. `release-status` compares the versions in rosdistro with the version
in your local `package.xml`.

## If the release repository was updated but opening the rosdistro pull request failed

Do not re-run a full `bloom-release`. The packaging branches and tags are already on `ros2-gbp/rtest-release`; running
the full command again duplicates work.

Bloom caches a GitHub credential separately from `gh` and git, in `~/.config/bloom` (plain JSON: `{"github_user": ..., "oauth_token": ...}`).
The cache is used without a validity check, so an expired token fails late, after the release repository has already been pushed.

1. Create a new token at <https://github.com/settings/tokens> with at least the `public_repo` scope. Add `workflow` if
you want bloom to update an existing fork of `ros/rosdistro`.
2. Remove `~/.config/bloom` so bloom prompts for the new token instead of reusing the stale cache.
3. Re-run with `--pull-request-only` so bloom skips straight to opening the pull request:

   ```bash
   pixi run -e release bloom-release rolling -- --pull-request-only
   ```

   Repeat for each distro whose pull request is missing.

4. When prompted `GitHub username [<something>]:`, type your actual GitHub login, not the default.

If you accept a wrong default, bloom builds the fork clone URL from that string regardless of which account the token
belongs to. It then fails with `repository '.../<wrong-name>/rosdistro.git' not found`, and it may create a real
(harmless, reusable) fork under your real account that it then cannot find. If that happens, this command shows where
the fork actually landed:

```bash
gh api repos/ros/rosdistro/forks --jq '.[] | select(.owner.login=="<your-real-login>")'
```

When bloom's git commands fail, the error line can include the raw OAuth token (`https://<token>:x-oauth-basic@github.com/...`).
Redact that before sharing terminal output. Treat any token that appears in an error message as compromised and revoke it.

## If you already bumped package.xml by hand

`prepare-release` would increment the version again (for example 0.2.4 to 0.2.5). Do not run it.

1. Make sure all three `package.xml` files have the same version.
2. In all three `CHANGELOG.rst` files, replace the `Forthcoming` heading with `X.Y.Z (YYYY-MM-DD)` using today's date.
Leave older sections unchanged.
3. Commit those files.
4. Create a git tag whose name is exactly that version (for example `git tag 0.2.4`) and push both the commit and the tag.
5. Continue from [step 6](#6-run-bloom-release-for-every-distro).

Next time, follow steps 1–7 instead so `prepare-release` owns the version bump and the tag.

## Optional: GitHub Release `.deb` packages

From the Actions tab, run **ros2-release.yml** via `workflow_dispatch`. That workflow attaches `.deb` files to a GitHub
Release. It does not update `packages.ros.org`. Skip it unless you specifically want those artifacts.

## Why `bloom-release.sh` does more than call `bloom-release`

`bloom-release` clones `ros2-gbp/rtest-release` into a throwaway `/tmp` directory on every run, which conflicts with
several common personal git setups. The wrapper works around those without changing your global git configuration:

- **No git identity in the `/tmp` clone.** If you set `user.name` / `user.email` per repository (for example with
`includeIf` in `~/.gitconfig`) rather than globally, bloom's internal `git commit --allow-empty` fails with "Author
identity unknown". The script copies this repository's identity into `GIT_AUTHOR_*` and `GIT_COMMITTER_*`.
- **HTTPS push with no credential helper.** The release repository is registered in rosdistro over `https://`, which
cannot push without a stored credential. The script overrides the push URL to SSH (`git@github.com:ros2-gbp/rtest-release.git`),
matching how this repository's `origin` is configured. Override that with `BLOOM_RELEASE_PUSH_URL` if the URL ever changes.
- **`commit.gpgsign = true` globally.** Bloom's bookkeeping commits cannot be signed: there is no secret key for a
repository bloom itself owns. The script disables `commit.gpgsign` and `tag.gpgsign` for that process only,
via `GIT_CONFIG_COUNT` / `GIT_CONFIG_KEY_n` / `GIT_CONFIG_VALUE_n`. Your global signing preference is left unchanged
everywhere else, including commits in this `rtest` repository.
