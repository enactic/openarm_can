# Development

## How to release

```bash
export LAUNCHPAD_UPLOADER_PGP_KEY=YOUR_PGP_KEY # e.g. export  LAUNCHPAD_UPLOADER_PGP_KEY=08D3564B7C6A9CAFBFF6A66791D18FCF079F8007
git clone https://github.com/apache/arrow.git
export APACHE_ARROW_REPOSITORY=${PWD}/arrow
git clone https://github.com/groonga/groonga.git
export GROONGA_REPOSITORY=${PWD}/groonga
git clone git@github.com:enactic/openarm_can.git
cd openarm_can
rake release NEW_VERSION=X.Y.Z # e.g. rake release 1.0.0
```
