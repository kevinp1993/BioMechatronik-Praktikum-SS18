#!/bin/bash

############################
###   HELPER FUNCTIONS   ###
############################

function absoluteToRelativePath {
  # both $1 and $2 are absolute paths beginning with /
  # stores the relative path from $1/START_PATH to $2/TARGET_PATH in $3/RESULT_PATH or echoes it

  START_PATH=$1
  TARGET_PATH=$2

  # check whether one of both paths ends with / and remove it for now
  ENDS_WITH_SLASH=false
  while [ "${START_PATH%"/"}" != "$START_PATH" ]; do
    START_PATH=${START_PATH::-1}
    ENDS_WITH_SLASH=true
  done
  while [ "${TARGET_PATH%"/"}" != "$TARGET_PATH" ]; do
    TARGET_PATH=${TARGET_PATH::-1}
    ENDS_WITH_SLASH=true
  done

  # initialize the common prefix and the relative path from START_PATH to TARGET_PATH
  COMMON_PREFIX=$START_PATH
  RELATIVE_PATH=""
  while [ "${TARGET_PATH#$COMMON_PREFIX}" == "$TARGET_PATH" ]; do
    # while the COMMOM_PREFIX is no substring of TARGET_PATH, reduce it
    COMMON_PREFIX="$(dirname $COMMON_PREFIX)"

    # thereby record the relative path
    if [ -z $RELATIVE_PATH ]; then
      RELATIVE_PATH=".."
    else
      RELATIVE_PATH="../$RELATIVE_PATH"
    fi
  done

  # if the reltiva path is empty, set it as the current directory
  if [ -z $RELATIVE_PATH ]; then
    RELATIVE_PATH="."
  fi

  if [ $COMMON_PREFIX == "/" ]; then
    # special case if only root is the common path
    RELATIVE_PATH="$RELATIVE_PATH/"
  fi

  # compute the unique part of the target path
  UNIQUE_POSTFIX="${TARGET_PATH#$COMMON_PREFIX}"

  # append the unique postfix to the relative path
  if [ -n $RELATIVE_PATH ] && [ -n $UNIQUE_POSTFIX ]; then
    RELATIVE_PATH="$RELATIVE_PATH$UNIQUE_POSTFIX"
  elif [ -n $UNIQUE_POSTFIX ]; then
    # remove the slash
    RELATIVE_PATH="${UNIQUE_POSTFIX:1}"
  fi

  # remove a prepending slash if any
  if [ "${RELATIVE_PATH#"/"}" != "$RELATIVE_PATH" ]; then
    RELATIVE_PATH=${RELATIVE_PATH:1}
  fi

  # if at least one input ended with a slash, add it to the result as well
  if [ $ENDS_WITH_SLASH == true ]; then
    RELATIVE_PATH="${RELATIVE_PATH}/"
  fi

  # return the result
  if [ $# -gt 2 ]; then
    eval $3="$RELATIVE_PATH"
  else
    echo $RELATIVE_PATH
  fi

  # clean up
  unset START_PATH
  unset TARGET_PATH
  unset ENDS_WITH_SLASH
  unset COMMON_PREFIX
  unset RELATIVE_PATH
  unset UNIQUE_POSTFIX
}


function cleanupVariables {

  unset HELP_FLAG
  unset CLEAN_FLAG
  unset WIPE_FLAG
  unset CHOS
  unset LR
  unset PM
  unset DWD
  unset ARG_LIST

  unset USER_DIR
  unset SCRIPT_DIR

  unset GCC_ARM_NONE_EABI_INCLUDE_PATH
  unset PROJECT_ROOT_PATH
  unset AMIROOS_ROOT_PATH
  unset CHIBIOS_ROOT_PATH

  unset CHIBIOS_PROJECT_PREFIX
  unset LIGHTRING_PROJECT_PREFIX
  unset POWERMANAGEMENT_PROJECT_PREFIX
  unset DIWHEELDRIVE_PROJECT_PREFIX

  unset PROJECT_ROOT_TO_CHIBIOS
  unset PROJECT_ROOT_TO_AMIROOS
}


#################
###   INTRO   ###
#################

echo "-------------------------------------------------------------------------------"
echo ""
echo "Setup to create QtCreator projects"
echo "=================================="
echo ""



##########################
###   INITIALIZATION   ###
##########################

# ignore case when comparing strings
shopt -s nocasematch

# detect what exactly should be done
HELP_FLAG="HELP"
CLEAN_FLAG="CLEAN"
WIPE_FLAG="WIPE"
CHIBIOS_FLAG="CHOS"
LIGHTRING_FLAG="LR"
POWERMANAGEMENT_FLAG="PM"
DIWHEELDRIVE_FLAG="DWD"

# start with an empty array
ARG_LIST=()
if [ $# == 0 ]; then
  # if no argument was specified, 'help' is considered as default
  ARG_LIST+=($HELP_FLAG)
else
  for ARG in "$@"; do
    case ${ARG,,} in
      "help"|"-help"|"--help")
        ARG_LIST+=($HELP_FLAG)
        ;;
      "clean")
        ARG_LIST+=($CLEAN_FLAG)
        ;;
      "wipe")
        ARG_LIST+=($WIPE_FLAG)
        ;;
      "all")
        ARG_LIST+=($LIGHTRING_FLAG $POWERMANAGEMENT_FLAG $DIWHEELDRIVE_FLAG)
        ;;
      "ChibiOS")
        ArrayContainsElement "${ARG_LIST[@]}" "$CHIBIOS_FLAG"
        if [ $? != 1 ]; then
          ARG_LIST+=($CHIBIOS_FLAG)
        fi
        ;;
      "LightRing")
        ArrayContainsElement "${ARG_LIST[@]}" "$LIGHTRING_FLAG"
        if [ $? != 1 ]; then
          ARG_LIST+=($LIGHTRING_FLAG)
        fi
        ;;
      "PowerManagement")
        ArrayContainsElement "${ARG_LIST[@]}" "$POWERMANAGEMENT_FLAG"
        if [ $? != 1 ]; then
          ARG_LIST+=($POWERMANAGEMENT_FLAG)
        fi
        ;;
      "DiWheelDrive")
        ArrayContainsElement "{$ARG_LIST[@]}" "DIWHEELDRIVE_FLAG"
        if [ $? != 1 ]; then
          ARG_LIST+=($DIWHEELDRIVE_FLAG)
        fi
        ;;
    esac
  done
fi



######################
###   PRINT HELP   ###
######################

if [[ "${ARG_LIST[@]}" == *"$HELP_FLAG"* ]]; then
    echo -e "> HELP"
    echo -e ""
    echo -e "\tThe following commands are available:"
    echo -e ""
    echo -e "\t  help            - Prints this help text."
    echo -e "\t  clean           - Deletes all files created by this script."
    echo -e "\t  wipe            - Deletes the .user files, created by QtCreator."
    echo -e "\t  ChibiOS         - Creates a project for ChibiOS."
    echo -e "\t  LightRing       - Creates a project for the LightRing module."
    echo -e "\t  PowerManagement - Creates a project for the PowerManagement module."
    echo -e "\t  DiWheelDrive    - Creates a project for the DiWheelDrive module."
    echo -e "\t  all             - Creates all three projects (no ChibiOS project)."
    echo -e ""
    echo -e "\tAny of these commands can be combined, e.g."
    echo -e "\t  $> ./setup.sh PowerManagement ChibiOS"
    echo -e "\twill create two projects."
    echo -e ""
    echo -e "-------------------------------------------------------------------------------"

    cleanupVariables
    exit
fi



#########################
###   CONFIGURATION   ###
#########################

USER_DIR=${PWD}
cd $(dirname ${BASH_SOURCE[0]})
SCRIPT_DIR=${PWD}
cd $USER_DIR

### the include path for GCC specific headers
GCC_ARM_NONE_EABI_INCLUDE_PATH="/opt/gcc-arm-none-eabi-4_8-2014q1/arm-none-eabi/include"
read -p "gcc_arm_none_eabi include path:     " -i "$GCC_ARM_NONE_EABI_INCLUDE_PATH" -e GCC_ARM_NONE_EABI_INCLUDE_PATH
# make absolute
if [ "/$GCC_ARM_NONE_EABI_INCLUDE_PATH" != "$GCC_ARM_NONE_EABI_INCLUDE_PATH" ]; then
  cd $GCC_ARM_NONE_EABI_INCLUDE_PATH
  GCC_ARM_NONE_EABI_INCLUDE_PATH=${PWD}
  cd $USER_DIR
fi
# append a slash if required
if [ "${GCC_ARM_NONE_EABI_INCLUDE_PATH%"/"}" == "$GCC_ARM_NONE_EABI_INCLUDE_PATH" ]; then
  GCC_ARM_NONE_EABI_INCLUDE_PATH="${GCC_ARM_NONE_EABI_INCLUDE_PATH}/"
fi

### the relative path where all project files shall be generated
cd "$SCRIPT_DIR/../../"
PROJECT_ROOT_PATH=${PWD}
cd $USER_DIR
read -p "path where to create project files: " -i "$PROJECT_ROOT_PATH" -e PROJECT_ROOT_PATH
# make absolute
if [ "/$PROJECT_ROOT_PATH" != "$PROJECT_ROOT_PATH" ]; then
  cd $PROJECT_ROOT_PATH
  PROJECT_ROOT_PATH=${PWD}
  cd $USER_DIR
fi
# append a slash if required
if [ "${PROJECT_ROOT_PATH%"/"}" == "$PROJECT_ROOT_PATH" ]; then
  PROJECT_ROOT_PATH="${PROJECT_ROOT_PATH}/"
fi

# the relative path to the AMiRo-OS root directory
cd "$SCRIPT_DIR/../../"
AMIROOS_ROOT_PATH=${PWD}
cd $USER_DIR
read -p "AMiRo-OS root path:                 " -i "$AMIROOS_ROOT_PATH" -e AMIROOS_ROOT_PATH
# make absolute
if [ "/$AMIROOS_ROOT_PATH" != "$AMIROOS_ROOT_PATH" ]; then
  cd $AMIROOS_ROOT_PATH
  AMIROOS_ROOT_PATH=${PWD}
  cd $USER_DIR
fi
# append a slash if required
if [ "${AMIROOS_ROOT_PATH%"/"}" == "$AMIROOS_ROOT_PATH" ]; then
  AMIROOS_ROOT_PATH="${AMIROOS_ROOT_PATH}/"
fi

# the relative path to the ChibiOS root directory
cd "${AMIROOS_ROOT_PATH}../ChibiOS/"
CHIBIOS_ROOT_PATH=${PWD}
cd $USER_DIR
read -p "ChibiOS root path:                  " -i "$CHIBIOS_ROOT_PATH" -e CHIBIOS_ROOT_PATH
# make absolute
if [ "/$CHIBIOS_ROOT_PATH" != "$CHIBIOS_ROOT_PATH" ]; then
  cd $CHIBIOS_ROOT_PATH
  CHIBIOS_ROOT_PATH=${PWD}
  cd $USER_DIR
fi
# append a slash if required
if [ "${CHIBIOS_ROOT_PATH%"/"}" == "$CHIBIOS_ROOT_PATH" ]; then
  CHIBIOS_ROOT_PATH="${CHIBIOS_ROOT_PATH}/"
fi

# the prefix names for the projects to be generated
CHIBIOS_PROJECT_PREFIX="ChibiOS"
LIGHTRING_PROJECT_PREFIX="LightRing"
POWERMANAGEMENT_PROJECT_PREFIX="PowerManagement"
DIWHEELDRIVE_PROJECT_PREFIX="DiWheelDrive"

echo ""



#############################
###   ENVIRONMENT SETUP   ###
#############################

# generate the relative path from the project root to ChibiOS
absoluteToRelativePath "$PROJECT_ROOT_PATH" "$CHIBIOS_ROOT_PATH" PROJECT_ROOT_TO_CHIBIOS
# generate the relative path from the project root to AMiRo-OS
absoluteToRelativePath "$PROJECT_ROOT_PATH" "$AMIROOS_ROOT_PATH" PROJECT_ROOT_TO_AMIROOS

# move back to the project root directory
cd $PROJECT_ROOT_PATH



######################
###   CLEAN STEP   ###
######################

# clean the project files
if [[ "${ARG_LIST[@]}" == *"$CLEAN_FLAG"* ]]; then
  echo -n "removing existing project files..."

  # remove all files
  rm -f ${CHIBIOS_PROJECT_PREFIX}.includes
  rm -f ${CHIBIOS_PROJECT_PREFIX}.files
  rm -f ${CHIBIOS_PROJECT_PREFIX}.config
  rm -f ${CHIBIOS_PROJECT_PREFIX}.creator

  rm -f ${LIGHTRING_PROJECT_PREFIX}.includes
  rm -f ${LIGHTRING_PROJECT_PREFIX}.files
  rm -f ${LIGHTRING_PROJECT_PREFIX}.config
  rm -f ${LIGHTRING_PROJECT_PREFIX}.creator

  rm -f ${POWERMANAGEMENT_PROJECT_PREFIX}.includes
  rm -f ${POWERMANAGEMENT_PROJECT_PREFIX}.files
  rm -f ${POWERMANAGEMENT_PROJECT_PREFIX}.config
  rm -f ${POWERMANAGEMENT_PROJECT_PREFIX}.creator

  rm -f ${DIWHEELDRIVE_PROJECT_PREFIX}.includes
  rm -f ${DIWHEELDRIVE_PROJECT_PREFIX}.files
  rm -f ${DIWHEELDRIVE_PROJECT_PREFIX}.config
  rm -f ${DIWHEELDRIVE_PROJECT_PREFIX}.creator

  echo -e "\tdone"
fi

# wipe even the user specific files (created by QtCreator)
if [[ "${ARG_LIST[@]}" == *"$WIPE_FLAG"* ]]; then
  echo -n "removing existing user specific project files..."

  # remove all user files
  rm -f ${CHIBIOS_PROJECT_PREFIX}.creator.user
  rm -f ${LIGHTRING_PROJECT_PREFIX}.creator.user
  rm -f ${POWERMANAGEMENT_PROJECT_PREFIX}.creator.user
  rm -f ${DIWHEELDRIVE_PROJECT_PREFIX}.creator.user

  echo -e "\tdone"
fi



#########################
###   CHIBIOS SETUP   ###
#########################

if [[ "${ARG_LIST[@]}" == *"$CHIBIOS_FLAG"* ]]; then
  echo -n "creating project files for ${CHIBIOS_PROJECT_PREFIX}..."

  # generate a file that contains all subdirectories as includes (but ignore hidden and documentation directories)
  find $PROJECT_ROOT_TO_CHIBIOS -type d | grep -v /[.][^.] | grep -v /doc/ > ${CHIBIOS_PROJECT_PREFIX}.includes
  # generate a file that specifies all files
  find $PROJECT_ROOT_TO_CHIBIOS -type f -iname "*.c" -o -iname "*.cpp" -o -iname "*.h" -o -iname "*.hpp" -o -iname "*.s" | grep -v /[.][^.] | grep -v /doc/ > ${CHIBIOS_PROJECT_PREFIX}.files
  # generate a default project configuration file if none exists so far
  if [ ! -f ${CHIBIOS_PROJECT_PREFIX}.config ]; then
    echo -e "// Add predefined macros for your project here. For example:\n// #define YOUR_CONFIGURATION belongs here\n" > ${CHIBIOS_PROJECT_PREFIX}.config
  fi
  # generate a default .creator file if none exists so far
  if [ ! -f ${CHIBIOS_PROJECT_PREFIX}.creator ]; then
    echo -e "[general]\n" > ${CHIBIOS_PROJECT_PREFIX}.creator
  fi

  echo -e "\tdone"
fi



###########################
###   LIGHTRING SETUP   ###
###########################

if [[ "${ARG_LIST[@]}" == *"$LIGHTRING_FLAG"* ]]; then
  echo -n "creating project files for ${LIGHTRING_PROJECT_PREFIX}..."

  # generate a file that contains all subdirectories as includes (but ignore hidden and documentation directories)
  echo $GCC_ARM_NONE_EABI_INCLUDE_PATH > ${LIGHTRING_PROJECT_PREFIX}.includes
  find $PROJECT_ROOT_TO_AMIROOS -type d | grep -v /[.][^.] | grep -v /doc/ | grep "devices\|include\|util\|boards\|components\|hal" | grep -iv "PowerManagement" | grep -iv "DiWheelDrive" >> ${LIGHTRING_PROJECT_PREFIX}.includes
  find $PROJECT_ROOT_TO_CHIBIOS -type d | grep -v /[.][^.] | grep -v /doc/ | grep /os/ >> ${LIGHTRING_PROJECT_PREFIX}.includes
  # generate a file that specifies all files
  find $PROJECT_ROOT_TO_AMIROOS -type f -iname "*.c" -o -iname "*.cpp" -o -iname "*.h" -o -iname "*.hpp" -o -iname "*.s" -o -iname "*.tpp" | grep -v /[.][^.] | grep -v /doc/ | grep -iv "PowerManagement" | grep -iv "DiWheelDrive" > ${LIGHTRING_PROJECT_PREFIX}.files
  find $PROJECT_ROOT_TO_CHIBIOS -type f -iname "*.c" -o -iname "*.cpp" -o -iname "*.h" -o -iname "*.hpp" -o -iname "*.s" | grep -v /[.][^.] | grep -v /doc/ | grep /os/ >> ${LIGHTRING_PROJECT_PREFIX}.files
  # generate a default project configuration file if none exists so far
  if [ ! -f ${LIGHTRING_PROJECT_PREFIX}.config ]; then
    echo -e "// Add predefined macros for your project here. For example:\n//#define YOUR_CONFIGURATION belongs here\n" > ${LIGHTRING_PROJECT_PREFIX}.config
  fi
  # generate a default .creator file if none exists so far
  if [ ! -f ${LIGHTRING_PROJECT_PREFIX}.creator ]; then
    echo -e "[general]\n" > ${LIGHTRING_PROJECT_PREFIX}.creator
  fi

  echo -e "\tdone"
fi



#################################
###   POWERMANAGEMENT SETUP   ###
#################################

if [[ "${ARG_LIST[@]}" == *"$POWERMANAGEMENT_FLAG"* ]]; then
  echo -n "creating project files for ${POWERMANAGEMENT_PROJECT_PREFIX}..."

  # generate a file that contains all subdirectories as includes (but ignore hidden and documentation directories)
  echo $GCC_ARM_NONE_EABI_INCLUDE_PATH > ${POWERMANAGEMENT_PROJECT_PREFIX}.includes
  find $PROJECT_ROOT_TO_AMIROOS -type d | grep -v /[.][^.] | grep -v /doc/ | grep "devices\|include\|util\|boards\|components\|hal" | grep -iv "LightRing" | grep -iv "DiWheelDrive" >> ${POWERMANAGEMENT_PROJECT_PREFIX}.includes
  find $PROJECT_ROOT_TO_CHIBIOS -type d | grep -v /[.][^.] | grep -v /doc/ >> ${POWERMANAGEMENT_PROJECT_PREFIX}.includes
  # generate a file that specifies all files
  find $PROJECT_ROOT_TO_AMIROOS -type f -iname "*.c" -o -iname "*.cpp" -o -iname "*.h" -o -iname "*.hpp" -o -iname "*.s" -o -iname "*.tpp" | grep -v /[.][^.] | grep -v /doc/ | grep -iv "LightRing" | grep -iv "DiWheelDrive" > ${POWERMANAGEMENT_PROJECT_PREFIX}.files
  find $PROJECT_ROOT_TO_CHIBIOS -type f -iname "*.c" -o -iname "*.cpp" -o -iname "*.h" -o -iname "*.hpp" -o -iname "*.s" | grep -v /[.][^.] | grep -v /doc/ | grep /os/ >> ${POWERMANAGEMENT_PROJECT_PREFIX}.files
  # generate a default project configuration file if none exists so far
  if [ ! -f ${POWERMANAGEMENT_PROJECT_PREFIX}.config ]; then
    echo -e "// Add predefined macros for your project here. For example:\n//#define YOUR_CONFIGURATION belongs here\n" > ${POWERMANAGEMENT_PROJECT_PREFIX}.config
  fi
  # generate a default .creator file if none exists so far
  if [ ! -f ${POWERMANAGEMENT_PROJECT_PREFIX}.creator ]; then
    echo -e "[general]\n" > ${POWERMANAGEMENT_PROJECT_PREFIX}.creator
  fi

  echo -e "\tdone"
fi



##############################
###   DIWHEELDRIVE SETUP   ###
##############################

if [[ "${ARG_LIST[@]}" == *"$DIWHEELDRIVE_FLAG"* ]]; then
  echo -n "creating project files for ${DIWHEELDRIVE_PROJECT_PREFIX}..."

  # generate a file that contains all subdirectories as includes (but ignore hidden and documentation directories)
  echo $GCC_ARM_NONE_EABI_INCLUDE_PATH > ${DIWHEELDRIVE_PROJECT_PREFIX}.includes
  find $PROJECT_ROOT_TO_AMIROOS -type d | grep -v /[.][^.] | grep -v /doc/ | grep "devices\|include\|util\|boards\|components\|hal" | grep -iv "LightRing" | grep -iv "PowerManagement" >> ${DIWHEELDRIVE_PROJECT_PREFIX}.includes
  find $PROJECT_ROOT_TO_CHIBIOS -type d | grep -v /[.][^.] | grep -v /doc/ >> ${DIWHEELDRIVE_PROJECT_PREFIX}.includes
  # generate a file that specifies all files
  find $PROJECT_ROOT_TO_AMIROOS -type f -iname "*.c" -o -iname "*.cpp" -o -iname "*.h" -o -iname "*.hpp" -o -iname "*.s" -o -iname "*.tpp" | grep -v /[.][^.] | grep -v /doc/ | grep -iv "LightRing" | grep -iv "PowerManagement" > ${DIWHEELDRIVE_PROJECT_PREFIX}.files
  find $PROJECT_ROOT_TO_CHIBIOS -type f -iname "*.c" -o -iname "*.cpp" -o -iname "*.h" -o -iname "*.hpp" -o -iname "*.s" | grep -v /[.][^.] | grep -v /doc/ | grep /os/ >> ${DIWHEELDRIVE_PROJECT_PREFIX}.files
  # generate a default project configuration file if none exists so far
  if [ ! -f ${DIWHEELDRIVE_PROJECT_PREFIX}.config ]; then
    echo -e "// Add predefined macros for your project here. For example:\n//#define YOUR_CONFIGURATION belongs here\n" > ${DIWHEELDRIVE_PROJECT_PREFIX}.config
  fi
  # generate a default .creator file if none exists so far
  if [ ! -f ${DIWHEELDRIVE_PROJECT_PREFIX}.creator ]; then
    echo -e "[general]\n" > ${DIWHEELDRIVE_PROJECT_PREFIX}.creator
  fi

  echo -e "\tdone"
fi



#################
###   OUTRO   ###
#################

cleanupVariables

echo ""
echo "-------------------------------------------------------------------------------"

