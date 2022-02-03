#!/bin/bash


if [ -d "debian" ] ; then
  cd debian
elif [[ -e "gen.sh " ]] ; then 
  cd ..
elif [ ! -e "packages-versioned"] ; then
  exit 1
fi

while IFS= read -r line
do
    I=$(expr $I + 1 )
    extension="${line##*.}"
    base="${line%.*}"
    case $base in
        major ) MAJOR=$extension ;;
        minor ) MINOR=$extension ;;
        patch ) PATCH=$extension ;;
    esac
done < "./scripts/version"

getArchFolder () {
    archFolder="*"
    if [[ -d "./tmp" ]] ; then
        var=$(find ./tmp/usr/lib/$archFolder$libdir/lib$libname$ext)
        for f in $var ; do 
            var=$f
            break
        done
        while [[ ! $(dirname $var) == "./tmp/usr/lib" ]] && [[ ${#var} -gt 1 ]] ; do 
            var=$(dirname $var)
        done 
        archFolder=$(basename $var) 
    fi
    echo "archfolder: $archFolder"
}


##########################################
#       Versioned packages               #
##########################################
while read p; do
    if [[ $p == "exit" ]] ; then
        break
    fi
    if [[ -z $p ]] ; then
        continue
    fi
    regex="^#"
    if [[ $p =~ $regex ]] ; then 
        continue
    fi
    echo 
    echo "############################################"
    echo $p

    line=$p
    pkgName=""  #name of the package
    libname=""  #name of the so\exe file without "lib" infront
    libdir=""
    targetLibdir=""
    library=""  #robworkpackage
    IncludeDir=""
    IncludeFile=""
    TargetIncludeDir=""
    type="new"
    noDev="false"
    noLib="false"
    noBin="true"
    gotSONAME="true"
    archFolder="*"
    unversioned="false"
    lintianOverride="package-name-doesnt-match-sonames"
    static="false"

    for elem in $p ; do 
        value="${elem##*=}"
        key="${elem%=*}"
        echo "key $key - value $value"
        if [[ $key == "pkg" ]] ; then
            pkgName=$value
        elif [[ $key == "static" ]] ; then
            static=$value
        elif [[ $key == "libname" ]] ; then
            libname=$value
        elif [[ $key == "libdir" ]] ; then
            libdir=$value
        elif [[ $key == "targetLibdir" ]] ; then
            targetLibdir=$value
        elif [[ $key == "lib" ]] ; then
            library=$value
        elif [[ $key == "dir" ]] ; then
            IncludeDir=$IncludeDir" "$value
        elif [[ $key == "nov" ]] ; then 
            unversioned=$value
        elif [[ $key == "inclfile" ]] ; then 
            IncludeFile=$IncludeFile" "$value
        elif [[ $key == "targetDir" ]] ; then 
            TargetIncludeDir=$value
        elif [[ $key == "type" ]] ; then
            type=$value
        elif [[ $key == "nodev" ]] ; then
            noDev=$value
        elif [[ $key == "nolib" ]] ; then
            noLib=$value
        elif [[ $key == "nobin" ]] ; then
            noBin=$value
        elif [[ $key == "gotsoname" ]] ; then
            gotSONAME=$value
        elif [[ $key == "lintian" ]] ; then 
            lintianOverride=$lintianOverride" "$value
        else
            echo "key $key not understood"
        fi
    done

    if [[ -z $pkgName ]] ; then
        echo "package not found"
        continue
    fi

    if [[ -z $libname ]] ; then 
        libname=$pkgName
    fi

    if [[ -z $library ]] ; then 
        library="robwork"
        if [ "${pkgName%_*}" == "sdurwhw" ] ; then
            library="robworkhardware"
        elif [ "${pkgName%_*}" == "sdurws" ] ; then
            library="robworkstudio"
        elif [ "${pkgName%_*}" == "sdurwsim" ] ; then
            library="robworksim"
        fi
    fi

    if [[ -z $IncludeDir ]] ; then 
        IncludeDir="/rw"
        if [ "${pkgName%_*}" == "sdurwhw" ] ; then
            IncludeDir="/rwhw"
        elif [ "${pkgName%_*}" == "sdurws" ] ; then
            IncludeDir="/rws"
        elif [ "${pkgName%_*}" == "sdurwsim" ] ; then
            IncludeDir="/rwsim"
        fi
        if [[ ! -z ${pkgName#*_} ]] ; then
            IncludeDir=$IncludeDir"libs/${pkgName#*_}"
        fi
    elif [[ $IncludeDir == "blank" ]]; then
        IncludeDir=""
    fi

    pkgName=${pkgName//_/-}
    echo
    echo "Working on pkg: $pkgName with libname: $libname, in library: $library, using includeDir: $IncludeDir"
    ############################################################
    #                           LIBS                           #
    ############################################################
    if [[ ! $noLib == "true" ]] ; then
        
        filename="lib$pkgName.install"

        if [[ $unversioned == "true" ]] ; then 
            filename="lib$pkgName.install"
        fi
        echo "Filename: $filename"

        if [[ $type == "new" ]] && [[ -e "$filename" ]] ; then
            rm "$filename"
        fi
        if [[ $static == "false" ]] ; then
            ext=".so.*"
            if [[ ! $gotSONAME == "true" ]] ; then 
                ext=".so"
            fi
        else 
            ext=".a"
        fi
        getArchFolder
        if [[ -z $targetLibdir ]] ; then 
            echo "usr/lib/$archFolder$libdir/lib$libname$ext" >> $filename
        else 
            echo "usr/lib/$archFolder$libdir/lib$libname$ext usr/lib/$archFolder$targetLibdir/lib$libname$ext" >> $filename
        fi
    fi
    ############################################################
    #                          DEV                             #
    ############################################################
    if [[ ! $noDev == "true" ]] ; then
        filename="lib$pkgName-dev.install"
        if [[ $type == "new" ]] && [[ -e "$filename" ]] ; then
            rm "$filename"
        fi
        if [[ $static == "false" ]] ; then
            if [[ $gotSONAME == "true" ]] ; then 
                echo "usr/lib/*$libdir/lib$libname.so" >> $filename
            fi
        else 
            ext=".a"
        fi

        if [[ -n $IncludeFile ]] ; then 
            for f in $IncludeFile ; do 
                if [[ -z $TargetIncludeDir ]] ; then 
                    echo "usr/include/$library-$MAJOR.$MINOR/$f" >> $filename
                else
                    echo "usr/include/$library-$MAJOR.$MINOR/$f usr/include/$library-$MAJOR.$MINOR$TargetIncludeDir/$(basename $f)" >> $filename
                fi
            done
        fi
        if [[ ! $IncludeDir =~ "none" ]] ; then
            for f in $IncludeDir ; do
                echo "usr/include/$library-$MAJOR.$MINOR$f/*" >> $filename
            done 
        fi

    fi

    ############################################################
    #                           BIN                            #
    ############################################################

    if [[ $noBin == "false" ]] ; then
        filename="$pkgName.install"
        if [[ $type == "new" ]] && [[ -e "$filename" ]] ; then
            rm "$filename"
        fi
        echo "usr/bin/$library" >> $filename
    fi

    ############################################################
    #                     lintian Override                     #
    ############################################################
    
    if [[ ! -z "$lintianOverride" ]] ; then
        
        filename="lib$pkgName.lintian-overrides"

        if [[ $unversioned == "true" ]] ; then 
            filename="lib$pkgName.lintian-overrides"
        fi
        echo "Filename: $filename"

        if [[ $type == "new" ]] && [[ -e "$filename" ]] ; then
            rm "$filename"
        fi

        for over in $lintianOverride ; do 
            echo $over >> $filename

        done
    fi

done < packages-install

