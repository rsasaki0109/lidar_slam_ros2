#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Validate rendered entry-page metadata without ROS or network access."""
import argparse
from html.parser import HTMLParser
import json
from pathlib import Path


class Metadata(HTMLParser):
    def __init__(self):
        super().__init__()
        self.title = ''
        self.in_title = False
        self.description = ''
        self.canonical = ''
        self.schemas = []
        self.script = None

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == 'title':
            self.in_title = True
        if tag == 'meta' and attrs.get('name') == 'description':
            self.description = attrs.get('content', '')
        if tag == 'link' and attrs.get('rel') == 'canonical':
            self.canonical = attrs.get('href', '')
        if tag == 'script' and attrs.get('type') == 'application/ld+json':
            self.script = ''

    def handle_data(self, data):
        if self.in_title:
            self.title += data
        if self.script is not None:
            self.script += data

    def handle_endtag(self, tag):
        if tag == 'title':
            self.in_title = False
        if tag == 'script' and self.script is not None:
            self.schemas.append(json.loads(self.script))
            self.script = None


def check(site):
    descriptions = set()
    for filename, language in [('index.html', 'en'), ('getting-started.html', 'en'),
                               ('getting-started-ja.html', 'ja'), ('comparison.html', 'en')]:
        meta = Metadata()
        meta.feed((site / filename).read_text(encoding='utf-8'))
        if not meta.title or not meta.description or meta.description in descriptions:
            raise ValueError(f'{filename}: missing title or unique description')
        if meta.title.count('lidar_slam Docs') > 1:
            raise ValueError(f'{filename}: repeated site name in title')
        descriptions.add(meta.description)
        if len(meta.schemas) != 1:
            raise ValueError(f'{filename}: expected one page schema')
        schema = meta.schemas[0]
        expected = 'https://rsasaki0109.github.io/lidar_slam/' + filename
        if meta.canonical != expected or schema.get('url') != meta.canonical:
            raise ValueError(f'{filename}: schema and canonical URL disagree')
        if schema.get('@type') != 'WebPage' or schema.get('@context') != 'https://schema.org':
            raise ValueError(f'{filename}: invalid page schema type or context')
        if schema.get('description') != meta.description or schema.get('inLanguage') != language:
            raise ValueError(f'{filename}: schema description or language disagrees')
    print('Verified metadata on four rendered entry pages.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--site-dir', type=Path, default=Path('site'))
    check(parser.parse_args().site_dir)
