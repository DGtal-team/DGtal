VERSION = '0.0.4'

def get_versions():
    """Returns versions for the DGtal Python package.

    from dgtalVersion import get_versions

    # Returns the DGtal repository version
    get_versions()['version']

    # Returns the package version. Since GitHub Releases do not support the '+'
    # character in file names, this does not contain the local version
    # identifier in nightly builds, i.e.
    #
    #  '4.11.0.dev20170208'
    #
    # instead of
    #
    #  '4.11.0.dev20170208+139.g922f2d9'
    get_versions()['package-version']
    """
    versions = {}
    versions['version'] = VERSION
    versions['package-version'] = VERSION.split('+')[0]
    return versions
