#!/bin/bash


######################################
#               Functions            #
######################################

echoActive=0
ECHO () {
    if [[ echoActive -eq 1 ]] ; then
        echo "$@"
    fi
}

getInstallFiles () {
    InstallFile=$1
    ret=""
    while IFS= read -r line; do
        ECHO "  Text read from file: $line"
        FILES="./tmp/"$line

        for f1 in $(find $FILES -type f) ; do
            value="${f1//'//'/'/'}" 
            ECHO "      File: $value"

            ret=$ret" "$value
        done
    done < "$InstallFile"
}



######################################
#               script               #
######################################


if [ -d "debian" ] ; then
  cd debian
elif [[ -e "gen.sh " ]] ; then 
  cd ..
elif [ ! -e "packages-versioned"] ; then
  exit 1
fi

pkgInstalled=""
pkgNotInstalled=""

allFiles=""
installFiles=""

#Find all files in exsistance
FILES="./tmp"
except="_lua"
for f in $(find $FILES -type f) ; do
    if [[ -e $f ]] && [[ ! $f =~ $except ]]; then 
        #ECHO $f 
        allFiles=$allFiles" "$f    
    fi
done 

#Find all install scripts
FILES="./*"
regexINSTALL="\.install$"
for f in $FILES ; do
    if [[ $f =~ $regexINSTALL ]] ; then
        #ECHO $f
        ret=""
        getInstallFiles $f
        installFiles=$installFiles" "$ret 
    fi
done

fileNotFound=""
dontInclude="example"
for f in $allFiles ; do
    
    if [[ ! $f =~ $dontInclude ]] ; then
        exsists=0
        for i in $installFiles ; do

            if [[ $i == $f ]] ; then 
                ECHO "fileFound: $i"
                exsists=1
                break
            fi
        done
        if [[ $exsists -eq 0 ]] ; then
            fileNotFound=$fileNotFound" "$f
            echo "File not found: $f"
        fi
    fi
done

