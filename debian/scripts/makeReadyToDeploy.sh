#!/bin/bash


######################################
#               Functions            #
######################################
readArg () {
    key=""
    doTestBuild="False"
    for var in "$@" ; do
        regex="^key"
        if [[ "$var" =~ $regex ]] ; then
            key=${var#*=}
        fi
        regex="^-T"
        if [[ "$var" =~ $regex ]] ; then
            doTestBuild="True"
        fi
    done
    echo "Using key: $key"
    echo "Do test build: $doTestBuild"
    
    #if RW_DEB_KEY is a global variable
    if [[ -z $key ]] && [[ ! -z $RW_DEB_KEY ]] ; then
        key=$RW_DEB_KEY
    fi
    
    if [[ $doTestBuild == "False" ]] ; then
        if [[ -z $key ]] ; then
            echo "A key for signing the package needs to be provided"
            echo "  key=ASDFISDK"
            exit 1
        fi
        
        if ! gpg -k $key ; then
            echo "Invalid key"
            exit 2
        fi
    fi

    
}

findScriptDir () {
    SOURCE="${BASH_SOURCE[0]}"
    while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
        TARGET="$(readlink "$SOURCE")"
        if [[ $TARGET == /* ]]; then
            SOURCE="$TARGET"
        else
            DIR="$( dirname "$SOURCE" )"
            SOURCE="$DIR/$TARGET" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
        fi
    done
    RDIR="$( dirname "$SOURCE" )"
    DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
}

saveVersion () {
    echo
    echo "New version $MAJOR.$MINOR.$PATCH-$DEBREV"
    echo
    echo "Storing new version"
    
    rm $input
    touch $input
    echo "major.$MAJOR" >> $input
    echo "minor.$MINOR" >> $input
    echo "patch.$PATCH" >> $input
    echo "debrev.$DEBREV" >> $input
    
    cd ${DIR}
    
    git add ./version
    git commit -m "Version number updated to $MAJOR.$MINOR.$PATCH-$DEBREV"
}

changeVersion () {
    input="$1"
    I=0
    if [ -e $input ] ; then
        while IFS= read -r line
        do
            I=$(expr $I + 1 )
            regex="^SET\($2"
            if [[ $line =~ $regex ]] ; then
                echo "$(basename $(dirname $input))/$(basename $input):$I: $line -> SET($2 $MAJOR.$MINOR.$PATCH)"
                break
            fi
        done < "$input"
        awk -v NRi="$I" -v vnum="SET($2 $MAJOR.$MINOR.$PATCH)" 'NR==NRi {$0=vnum} 1' $input > ./tmp && mv tmp $input
    else
        echo "FILE NOT FOUND $input"
    fi
}

handleMergeConflicts () {
    result=0
    conflicts=$(git diff --name-only --diff-filter=U)
    for con in $conflicts ; do
        nano $con
        git add $con
    done
    git merge --continue && result=1
    
    if [ $result -eq 0 ] ; then
        echo "Somthing went wrong during merge of upstream into master"
        exit 6
    fi
    
    
}

makeSourcePkg () {
    
    result=1
    gbp dch -R -N "$MAJOR.$MINOR.$PATCH-$DEBREV" --ignore-branch --upstream-branch=master
    git add -A
    git commit -m "Updated Changelog to version $MAJOR.$MINOR.$PATCH-$DEBREV"
    git push || exit 1
    
    rm -r ../build
    gbp buildpackage --git-tag --git-export-dir=../.rw_deb -S --git-upstream-tree=SLOPPY --git-ignore-branch  --no-sign || result=0
    if [ $result -eq 1 ] ; then
        debsign -k $key ../.rw_deb/robwork_${MAJOR}.${MINOR}.${PATCH}-${DEBREV}_source.changes || result=0
        if [ $result -eq 1 ] ; then
            echo Succes
        else
            echo "Failed to sign package"
            exit 4
        fi
    else
        echo "Somthing went wrong during source build"
        exit 5
    fi
    
}

uploadPackage () {
    read -p "do you want to upload this package? [yN]" -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]] ; then
        echo "Package not uploaded"
    else
        read -p "Are you sure? [yN]" -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]] ; then
            echo "Package not uploaded"
        else
            dput robwork $rootDir/../.rw_deb/robwork_$MAJOR.$MINOR.$PATCH-${DEBREV}_source.changes
        fi
    fi
}

######################################
#               script               #
######################################

readArg $@ #read arguments

findScriptDir #find the directory of the script
echo "Found scribt in directory: '$DIR'"

#################### Get Version number ########################
input="$DIR/version"
MAJOR=0
MINOR=0
PATCH=0
DEBREV=1

I=0
while IFS= read -r line
do
    I=$(expr $I + 1 )
    extension="${line##*.}"
    base="${line%.*}"
    case $base in
        major ) MAJOR=$extension ;;
        minor ) MINOR=$extension ;;
        patch ) PATCH=$extension ;;
        debrev ) DEBREV=$extension ;;
    esac
done < "$input"

if [ ! $I -gt 3 ] ; then
    "Missing newline at end of $input - aborting"
    exit 2
fi
oldmj=$MAJOR
oldmi=$MINOR
oldpt=$PATCH
olddr=$DEBREV

#################### Update Version number ########################

if [[ $doTestBuild == "False" ]] ; then
    echo "Current version $MAJOR.$MINOR.$PATCH"
    echo "Trying to decide new Version number"
    echo "  MAJOR version when you make incompatible API changes"
    echo "  MINOR version when you add functionality in a backwards compatible manner"
    echo "  PATCH version when you make backwards compatible bug fixes"
    echo "  DEBREV version when only the debian folder has been changed"
    
    isDebRev=0
    echo
    read -p "Is this a DEBREV change? [yN]" -n 1 -r
    if [[ ! $REPLY =~ ^[Yy]$ ]] ; then
        echo
        read -p "Is this a PATCH change? [yN]" -n 1 -r
        if [[ ! $REPLY =~ ^[Yy]$ ]] ; then
            echo
            read -p "Is this a MINOR change? [yN]" -n 1 -r
            if [[ ! $REPLY =~ ^[Yy]$ ]] ; then
                echo
                read -p "Is this a MAJOR change? [yN]" -n 1 -r
                if [[ ! $REPLY =~ ^[Yy]$ ]] ; then
                    echo
                    echo "No Version Change Can't make update"
                    exit 3
                else
                    MAJOR=$(expr $MAJOR + 1)
                    MINOR=0
                    PATCH=0
                    DEBREV=1
                fi
            else
                MINOR=$(expr $MINOR + 1)
                PATCH=0
                DEBREV=1
            fi
        else
            PATCH=$(expr $PATCH + 1)
            DEBREV=1
        fi
    else
        DEBREV=$(expr $DEBREV + 1)
        isDebRev=1
    fi
    
    #################### Save Version number ########################
    
    saveVersion #Save version number and update the control file then commit the changes
    cd ../..

    #################### Change RobWork Version Number ################
    rootDir=$(pwd)

    tagName="Debian/$MAJOR.$MINOR.$PATCH"
    echo "TAG: $tagName"

    makeSourcePkg
    uploadPackage
    
else
    echo "Making testbuild"
    result=0

    gbp buildpackage --git-export-dir=../.rw_deb --git-upstream-tree=SLOPPY --git-debian-branch=debian-pkg --no-sign -S || result=1
    
    if [[ ! -e ../robwork_$MAJOR.$MINOR.$PATCH.orig.tar.gz && $result -eq 0 ]] ; then
        cp ../.rw_deb/robwork_$MAJOR.$MINOR.$PATCH.orig.tar.gz ../robwork_$MAJOR.$MINOR.$PATCH.orig.tar.gz
    fi
    if [[ $result -eq 0 ]] ; then
        debuild --no-sign || result=1
    fi
    
    
    exit $result
fi

exit 0

#4DD3D1A9